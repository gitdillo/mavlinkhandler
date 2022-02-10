from pymavlink import mavutil
import os
import time
import threading

class ErrorException(Exception):
    """Base class for other exceptions"""
    pass

class TimeoutException(ErrorException):
    pass


# Not used. Instead, we use optional args mavlink2 and dialect in the constructor of MavlinkHandler
# def set_mavlink2():
#     '''
#     Sets mavutil to use MAVLINK2, ardupilotmega dialect
#     '''
#     os.environ['MAVLINK20'] = '1'
#     mavutil.set_dialect('ardupilotmega')


class SourceHistory(object):
    def __init__(self, history_depth=10, logger=None, source_system=None, source_component=None,
                 verbose_message_drop=False, verbose_new_messages=False):
        self.history_depth = history_depth
        self.logger = logger
        self.source_system = source_system
        self.source_component = source_component
        self.verbose_message_drop = verbose_message_drop
        self.verbose_new_messages = verbose_new_messages
        self.mavlink_messages = {}
        self.last_received_seq = None
        self.total_received_messages = 0
        self.total_dropped_messages = 0

        # if self.logger is None:
        #     self.logger = logging.getLogger('source_history_logger')
        #     self.logger.setLevel('DEBUG')
        #     self.logger.addHandler(logging.StreamHandler())

    def store_message(self, msg):
        '''
        Stores passed msg to field "mavlink_messages", which is a dict of all message types received.
        If the message is of a type not among the dict's keys, a new key is added with this message type.
        The entries inside a message type are a list of dicts with each dict containing "message" (the message) and
        "timestamp" (current time).
        '''
        if not (self.source_system == msg.get_srcSystem() and self.source_component == msg.get_srcComponent()):
            raise ValueError('Asked to store message whose system or component id do not match those of this history')

        self.update_message_seq(msg.get_seq())

        if msg.get_type() not in self.mavlink_messages.keys():
            if self.verbose_new_messages:
                s = "Source " + str(self.source_system) + ':' + str(
                    self.source_component) + ", adding mavlink message type: " + str(msg.get_type())
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            self.mavlink_messages[msg.get_type()] = [{'message': msg, 'timestamp': time.time()}]
        else:
            while len(self.mavlink_messages[msg.get_type()]) >= self.history_depth:
                self.mavlink_messages[msg.get_type()].pop()
            self.mavlink_messages[msg.get_type()].insert(0, {'message': msg, 'timestamp': time.time()})

    def get_message(self, msg_type, msg_index=0):
        '''
        Returns a message of type "msg_type" from index "msg_index" of this source history.

        Throws:
            KeyError: if the "msg_type" does not exist
            IndexError: if there is no message at the required "msg_index"
        '''
        return self.mavlink_messages[msg_type][msg_index]

    def update_message_seq(self, seq):
        '''
        Reads the passed message's seq and records it in field "last_received_seq", updates total messages received and
        checks if any messages were dropped in between
        '''
        # If self.last_received_seq is None this is our fist message so we need to init
        if self.last_received_seq is None:
            self.last_received_seq = seq
            self.total_received_messages = 1
            return

        # If the seq of the message is LOWER than our last received message, we have wrapped around (0 <= seq <= 255)
        if seq < self.last_received_seq:
            dropped_messages = 255 - self.last_received_seq + seq
        else:
            dropped_messages = seq - self.last_received_seq - 1

        # Report detection of dropped messages
        if self.verbose_message_drop and not dropped_messages == 0:
            s = 'Detected ' + str(dropped_messages) + ' dropped messages from source ' + str(
                self.source_system) + ':' + str(self.source_component) + '. Previous/current seq: ' + str(self.last_received_seq) + '/' + str(seq)
            if self.logger is not None:
                self.logger.info(s)
            else:
                print(s)

        # Update our books
        self.total_dropped_messages += dropped_messages
        self.total_received_messages += 1
        self.last_received_seq = seq

    def get_message_rate_Hz(self, msg_type):
        '''
        Returns the Hz rate for a given message.

        Raises KeyError if the message type has not been received yet.

        Returns None if the message type exists but we haven't received >2 messages yet.
        '''
        if msg_type not in self.mavlink_messages.keys():
            raise KeyError('Message type: "' + str(msg_type) + '" has not been received')
        timestamps = [t['timestamp'] for t in self.mavlink_messages[msg_type]]
        if len(timestamps) < 2:
            return None
        diffs = []
        for i in range(len(timestamps)-1):
            diffs.append(timestamps[i] - timestamps[i+1])
        return len(diffs) / sum(diffs)

    def get_message_type_list(self):
        return list(self.mavlink_messages.keys())


class MavlinkHistory(object):
    '''
    Contained inside a "MavlinkHandler" object, in its "history" field.

    Contains source history objects of type "SourceHistory" under field "source_histories" (list)

    Incoming messages are passed to its "store_message()" method, which then sorts them out to the appropriate source
    history according to their sender's id.
    '''
    def __init__(self, history_depth=10, logger=None, verbose_message_drop=False, verbose_new_messages=False):
        self.history_depth = history_depth
        self.source_histories = []
        self.logger = logger
        self.mavlink_update_thread = None
        self.message_request_list = []
        self.total_messages_received = 0
        self.total_messages_sent = 0
        self.verbose_message_drop = verbose_message_drop
        self.verbose_new_messages = verbose_new_messages

    class MessageRequest(object):
        '''
        Class for storing message requests as used by method "get_next_message()".
        A message request contains filters (message_type, system_id, component_id) which must be met in order for the
        request to be met.
        Requests are stored in a "MavlinkHistory" object's "message_request_list" field and are checked by the
        "MavlinkHistory" object's "store_message()" method. "store_message()" is a hook to the mavlink update thread and
        gets called every time a new message is intercepted by the thread.
        '''
        def __init__(self, message_type=None, system_id=None, component_id=None):
            self.message_type = message_type
            self.system_id = system_id
            self.component_id = component_id
            self.creation_timestamp = time.time()
            self.message = None
            self.message_timestamp = None

        def check_message(self, msg):
            '''
            Checks if a particular message passes the test criteria for this MessageRequest and returns False if it
            doesn't. If it does, sets the message to this object's ".message" field, adds a timestamp in this object's
            ".message_timestamp" field and returns True.
            '''
            # First, check if the message is filteredd out
            if self.message_type is not None and not self.message_type == msg.get_type():
                return False
            if self.system_id is not None and not msg.get_srcSystem() == self.system_id:
                return False
            if self.component_id is not None and not msg.get_srcComponent() == self.component_id:
                return False
            # If we reach here, the message checks out
            self.message = msg
            self.message_timestamp = time.time()
            return True

    def add_message_request(self, message_request):
        self.message_request_list.append(message_request)

    def remove_message_request(self, message_request):
        if message_request in self.message_request_list:
            self.message_request_list.remove(message_request)

    def get_source_history(self, system_id, component_id):
        '''
        Returns one of this object's source histories given a source system and component.
        If this source system and component do not match an existing history, returns None
        '''
        for s in self.source_histories:
            if s.source_system == system_id and s.source_component == component_id:
                return s
        return None

    def add_source_history(self, system_id, component_id, verbose=True):
        '''
        Adds a source history with passed system_id, component_id to this object's "source_histories" field. Returns a
        reference to the added SourceHistory object.
        If a source history with these ids already exists, adds nothing and returns None.
        '''
        # Check that the requested source history does not actually exists
        if self.get_source_history(system_id, component_id) is None:
            if verbose:
                s = 'Adding source with system id: ' + str(system_id) + ', component id: ' + str(component_id)
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            history = SourceHistory(history_depth=self.history_depth, logger=self.logger, source_system=system_id,
                                    source_component=component_id, verbose_message_drop=self.verbose_message_drop,
                                    verbose_new_messages=self.verbose_new_messages)
            self.source_histories.append(history)
            return history
        else:
            if verbose:
                s = 'add_source_history(): source history with system id: ' + str(system_id) + ', component id: ' + str(
                        component_id) + ' already exists. Skipping'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            return None

    def get_source_ids(self):
        '''
        Returns list of dicts with keys "system", "component" for each source in this object's "source_histories" field.
        '''
        l = []
        for s in self.source_histories:
            l.append({'system': s.source_system, 'component': s.source_component})
        return l

    def store_message(self, msg, verbose=True):
        '''
        Stores the passed message to the appropriate source history. If a source history with the source system and
        component of the message does not exist, it is added to this object's source histories.
        '''
        source_history = self.get_source_history(msg.get_srcSystem(), msg.get_srcComponent())
        if source_history is None:
            source_history = self.add_source_history(msg.get_srcSystem(), msg.get_srcComponent(), verbose=verbose)
        source_history.store_message(msg)
        self.total_messages_received += 1

        for request in self.message_request_list:
            if request.message is None:
                if request.check_message(msg):
                    self.remove_message_request(request)

    def get_message(self, system_id, component_id, msg_type, msg_index=0, verbose=False):
        '''
        Returns message of "msg_type" from source with ids "system_id", "component_id".
        Returned value is a dict with keys "message", containing the message and "timestamp", containing the message's
            time of arrival.
        If "msg_index" is passed, returns message at that index, otherwise returns the last received message.
        If a source history with the passed ids passed does not exist, returns None.

        Throws:
            KeyError: if the "msg_type" does not exist
            IndexError: if there is no message at the required "msg_index"
        '''
        history = self.get_source_history(system_id, component_id)
        if history is None:
            if verbose:
                s = 'MavlinkHistory.get_message(): ' + 'source history with system id: ' + str(
                    system_id) + ', component id: ' + str(component_id) + ' does not exist'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            return None
        return history.get_message(msg_type, msg_index=msg_index)

    def get_last_message(self, message_type, system_id=None, component_id=None, verbose=False):
        '''
        Returns the last received message of type "message_type"
        "message_type" must be a string as returned by the "get_type()" method of mavlink messages

        Optional args system_id and component_id can be left None, if there is only a single mavlink source present.
        If however, more than one source has been registered and these args are not defined, this method will raise
        TypeError.

        See also: get_next_message()
        '''
        # Check that if we have no system/component ids then we only have one source
        if len(self.source_histories) > 1 and (system_id is None or component_id is None):
            if verbose:
                s = 'MavlinkHistory.get_last_message(): multiple sources exist but no system/component ids were provided'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            raise TypeError('No system_id or component_id provided but more than one source has been registered.')

        if system_id is None and component_id is None:
            history = self.source_histories[0]
        else:
            history = self.get_source_history(system_id, component_id)

        return history.get_message(message_type, msg_index=0)

    def get_next_message(self, message_type=None, system_id=None, component_id=None, timeout_sec=5,
                         verbose=False, loop_period_sec=0.01, blocking=True):
        '''
        Returns the next occurence of a message, None, if no suitable message is received within "timeout_sec" or a
        MessageRequest object if the optional arg "blocking" is set to False.

        Messages are filtered by three optional args:
            "message_type": string as returned by the "get_type()" method of mavlink messages
            "system_id": integer, source system id
            "component_id": integer, source component id
        If any of these is left None, it is skipped. If all are None, the next message to be received is returned.

        If "blocking" is set to False, a MessageRequest object is returned. This will be automatically filled by the
        MavlinkUpdateThread (assuming it is running). When it is filled, its "message" field will hold the message.
        Until it is filled, it is None.
        A calling function that does not want to block until a suitable message is received can set "blocking=False",
        get the resulting MessageRequest and then periodically check its "message" field until it gets a non None value.

        WARNING:
            Optional arg "timeout_sec" can be set to None but this will make this method block indefinitely if no
            suitable message appears in the stream. Use with extreme caution.

        See also: get_last_message()
        '''
        # Check that if we have no system/component ids then we only have one source
        if len(self.source_histories) > 1 and (system_id is None or component_id is None):
            if verbose:
                s = 'MavlinkHistory.get_last_message(): multiple sources exist but no system/component ids were provided'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            raise TypeError('No system_id or component_id provided but more than one source has been registered.')

        request = self.MessageRequest(message_type, system_id=system_id, component_id=component_id)
        # the line below adds a request to this object. This will be updated via this object's store_message(), which is
        # attached as a hook to the update thread
        self.add_message_request(request)

        if not blocking:
            return request

        t0 = time.time()
        while True:
            if timeout_sec is not None and time.time() - t0 > timeout_sec:
                if verbose:
                    s = 'Timeout after waiting for ' + str(timeout_sec) + ' seconds for message of type: ' + str(
                        message_type) + ' from system_id: ' + str(system_id) + ', component id: ' + str(component_id)
                    if self.logger is None:
                        print(s)
                    else:
                        self.logger.info(s)
                self.remove_message_request(request)
                return None
            if request.message is not None:
                # self.remove_message_request(request)
                return request.message
            time.sleep(loop_period_sec)

    def wait_heartbeat(self, timeout_sec=5, verbose=False):
        '''
        Convenience method to ensure we have a heartbeat in our stream. Does not do anything other than block until a
        heartbeat is received.
        Internally, simply calls this object's "get_next_message()" with a HEARTBEAT argument.

        Returns the heartbeat message or None if nothing received within timeout_sec.

        The caller can examine the returned heartbeat's get_srcSystem() and get_srcComponent() if setting a target
        system / component is needed.
        '''
        return self.get_next_message('HEARTBEAT', verbose=verbose, timeout_sec=timeout_sec, blocking=True)

    def get_source_history_message_types(self, system_id, component_id):
        return self.get_source_history(system_id, component_id).get_message_type_list()

class MavlinkUpdateThread(object):

    def __init__(self, connection, logger=None, name='MavlinkUpdateThread', hook_list=[], update_loop_period_sec=0.01,
                 mavlinkhandler_ref=None):

        self.logger = logger
        self.name = name
        self.connection = connection
        self.hook_list = hook_list
        self.update_loop_period_sec = update_loop_period_sec

        self.thread = None
        self.isPaused = False
        self.isRunning = False
        self.termflag = threading.Event()
        self.pauseflag = threading.Event()

        self.mavlinkhandler_ref = mavlinkhandler_ref

    def add_hook(self, function):
        '''
        Adds a hook to the mavlink update thread.
        The hooks are called each time a message is received in the thread and are passed the received message.

        If the argument is not callable, throws ValueError.
        If the argument already exists, it is not added.
        '''
        if not callable(function):
            raise ValueError('Hook must be callable.')
        if function not in self.hook_list:
            self.hook_list.append(function)

    def remove_hook(self, function):
        if function in self.hook_list:
            self.hook_list.remove(function)

    def clear_hooks(self):
        self.hook_list = []

    def start(self, verbose=True):
        self.thread = threading.Thread(target=self.update,
                             kwargs={'message_timeout': 5,
                                     'verbose': verbose})

        self.thread.daemon = True
        self.thread.start()
        self.isPaused = False
        self.isRunning = True

    def pause(self, timeout=5, verbose=True):
        self.pauseflag.set()
        request_time = time.time()
        while True:
            if self.isPaused:
                return True
            if time.time() - request_time > timeout:
                raise TimeoutException(self.name + ': timeout while waiting for mavlink update thread to pause.')
            time.sleep(.01)

    def resume(self, timeout=5, verbose=True):
        self.pauseflag.clear()
        request_time = time.time()
        while True:
            if not self.isPaused:
                return True
            if time.time() - request_time > timeout:
                raise TimeoutException(self.name + ': timeout while waiting for mavlink update thread to resume.')
            time.sleep(.01)

    def terminate(self):
        self.termflag.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join()

    def update(self, message_timeout=5, verbose=True):

        while True:

            t_start = time.time()

            if self.termflag.isSet():
                s = str(self.name) + ': MAVLINK update thread exiting.'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
                self.isPaused = False
                self.isRunning = False
                return

            if self.pauseflag.isSet():
                if self.isPaused == False:
                    self.isPaused = True
                    self.isRunning = False
                    if verbose:
                        s = str(self.name) + ': pausing mavlink update thread'
                        if self.logger is not None:
                            self.logger.info()
                        else:
                            print(s)
                time.sleep(self.update_loop_period_sec)
                continue
            else:
                if self.isPaused:
                    self.isPaused = False
                    self.isRunning = True
                    if verbose:
                        s = str(self.name) + ': resuming mavlink update thread'
                        if self.logger is not None:
                            self.logger.info(s)
                        else:
                            print(s)

            # Reaching here means we are neither terminated not paused
            msg = self.connection.recv_match(blocking=True, timeout=message_timeout)
            if msg is None:
                if verbose:
                    s = str(self.name) + ': ' + str(
                        message_timeout) + ' second timeout while waiting for MAVLINK message'
                    if self.logger is not None:
                        self.logger.info(s)
                    else:
                        print(s)
            else:
                for function in self.hook_list:
                    function(msg, self.mavlinkhandler_ref)

            sleep_time = self.update_loop_period_sec - (time.time() - t_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

class MavlinkHandler(object):

    def __init__(self, connection_string='udpin:0.0.0.0:14550', baud=57600, mavlink2=True, dialect='ardupilotmega',
                 logger=None, name='MavlinkHandler', thread_name=None, thread_logger=None, thread_start=False,
                 mavlink_history_depth=10, verbose_thread=False, verbose_message_drop=False):

        self.connection_string = connection_string
        self.baud = baud
        self.mavlink2 = mavlink2
        self.dialect = dialect
        self.logger = logger
        self.name = name
        self.altered_source = False     # signals temporary change of source ids to forward a message, see send_message()
        self.verbose_thread = verbose_thread

        self.connection = None     # this will hold the mavlink connection

        self.history = MavlinkHistory(history_depth=mavlink_history_depth, logger=self.logger,
                                      verbose_message_drop=verbose_message_drop)

        # NOTE: we add this object's history's store_message method as an initial hook to the mavlink update thread
        self.attach_mavlink_update_thread(name=thread_name, hook_list=[self.history.store_message],
                                          start_thread=thread_start, logger=thread_logger, verbose=self.verbose_thread)

        if self.mavlink2:
            os.environ['MAVLINK20'] = '1'

        mavutil.set_dialect(self.dialect)

    def attach_mavlink_update_thread(self, logger=None, name='MavlinkUpdateThread', hook_list=[], verbose=True,
                                     start_thread=True):
        # if we have not set "mav", there is no connection yet so the thread cannot start
        if self.connection is None and start_thread is True:
            start_thread = False
            if verbose:
                s = str(
                    self.name) + ': attach_mavlink_update_thread(): cannot start mavlink update thread because connection has not been set yet. Thread attached but not started.'
                if self.logger is None:
                    print(s)
                else:
                    self.logger.info(s)

        self.mavlink_update_thread = MavlinkUpdateThread(self.connection, logger=logger, name=name, hook_list=hook_list,
                                                         mavlinkhandler_ref=self)
        if start_thread:
            self.mavlink_update_thread.start(verbose=verbose)

    def set_source(self, system_id, component_id):
        '''
        If connection (field .mav of this mav) has been initialised (via this object's connect() method), sets its
        .srcSystem / .srcComponent fields to the passed ids
        '''
        if self.connection is not None:
            self.connection.mav.srcSystem = system_id
            self.connection.mav.srcComponent = component_id


    def send_message(self, message, preserve_source=False, verbose=True,
                     force_mavlink1=False, wait_altered_source_clear_seconds=1, wait_loop_period_seconds=0.001):
        '''
        Sends a mavlink message, assuming a connection has been set up (field "connection" is not None).

        If preserve_source is set to True, the passed message will be transmitted as is, i.e. using exists in its
        _msgbuf field, which includes its original source system and source component, rather than that of this object
        (which can be found at self.connection.mav.srcSystem and self.connection.mav.srcComponent).
        The idea is for preserve_source to be used when retransmitting a message received from another source, e.g.
        retransmitting a HEARTBEAT received by some component on the local system.

        NOTE 1: when preserve_source is True, sending the messages will not increment this object's seq
            (self.connection.mav.seq), which is included in the metadata of messages the object sends.
        NOTE 2: STATUS_TEXT is a special case to the above, since for some unknown reason, these have empty _msgbuf. The
            workaround employed here is that the object is sent using the normal send() method of this object
            (self.connection.mav.send()) which packs the message. To avoid packing using this objects system / component
            ids, this method sets them to those of the message we are transmitting, transmits, then resets them to the
            original.
            To ensure that no other call to the object happens while transmitting with these changed ids, the object
            raises flag self.altered_source (True) to signal that the object is transmitting with sys/comp ids other
            than its own. Other calls to this method while this flag is True, will wait, checking every
            wait_loop_period_seconds until it either drops or raises TimeoutException if the flag doesn't clear after
            wait_altered_source_clear_seconds.
            Just like the other cases, sending STATUS_TEXT with preserve_source does not increment this object's
            self.connection.mav.seq.
        '''
        if self.connection is None:
            if verbose:
                s = str(self.name) + ': cannot send since connection has not yet been set'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            return

        # If the object is already sending with altered_source, wait till it clears
        t0 = time.time()
        while self.altered_source:  # wait for altered_source flag to clear
            if time.time() - t0 > wait_altered_source_clear_seconds:
                raise TimeoutException(str(self.name) + ': Timeout waiting for end of sending as altered source.')
            time.sleep(wait_loop_period_seconds)

        # When asked to preserve source, write the message buffer directly. Code has been modified from
        # ardupilotmega.py, MAVLink.send()
        if preserve_source:
            # NOTE!!! for some weird reason, STATUSTEXT messages have an empty message buffer. We preserve source by
            # modifying our connection. This can result in source mixup if this method is accessed simultaneously by
            # different callers, which is why we need to use self.altered_source flag
            if message._msgbuf is None and message.get_type() == 'STATUSTEXT':
                original_source = (self.connection.mav.srcSystem, self.connection.mav.srcComponent)
                self.altered_source = True      # signal that we are transmitting with different source ids
                self.set_source(message.get_srcSystem(), message.get_srcComponent())
                self.connection.mav.send(message)
                self.connection.mav.seq = (self.connection.mav.seq - 1) % 256   # restore the seq incremented by self.connection.mav.seq
                self.set_source(original_source[0], original_source[1])
                self.altered_source = False  # signal that we have finished sending with different source ids
            else:
                # In this case, we have a non STATUS_TEXT message so we use the code from ardupilotmega.py,
                # MAVLink.send() to write its buffer directly to self.connection.mav.file.write()
                self.connection.mav.file.write(bytes(message._msgbuf))
                # self.connection.mav.seq = (self.connection.mav.seq + 1) % 256 # no seq increment with preserve_source
                self.connection.mav.total_packets_sent += 1
                self.connection.mav.total_bytes_sent += len(bytes(message._msgbuf))
            if self.connection.mav.send_callback:
                self.connection.mav.send_callback(message, *self.connection.mav.send_callback_args,
                                                  **self.connection.mav.send_callback_kwargs)
        else:
            self.connection.mav.send(message)
        self.history.total_messages_sent += 1

    def setup_connection(self, connection_string=None, baud=None, source_system=255, source_component=0,
                         planner_format=None, write=False, append=False, robust_parsing=True, notimestamps=False,
                         input=True, dialect=None, autoreconnect=False, zero_time_base=False, retries=3,
                         use_native=False, force_connected=False, progress_callback=None, verbose=True):
        '''
        Do not use directly, rather use connect() which calls this one.
        '''

        if connection_string is not None:
            if verbose:
                s = str(self.name) + ': setup_connection: setting connection string to ' + str(connection_string)
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)

        if self.connection_string is None:
            raise ValueError('Cannot set connection because connection_string not set')

        self.connection = mavutil.mavlink_connection(self.connection_string, baud=baud, source_system=source_system,
                                                     source_component=source_component, planner_format=planner_format,
                                                     write=write, append=append, robust_parsing=robust_parsing,
                                                     notimestamps=notimestamps, input=input, dialect=dialect,
                                                     autoreconnect=autoreconnect, zero_time_base=zero_time_base,
                                                     retries=retries, use_native=use_native,
                                                     force_connected=force_connected,
                                                     progress_callback=progress_callback)

        if verbose:
            s = 'Setting up connection with connection string: ' + self.connection_string + '\nbaud: ' + str(
                baud) + '\nsource system: ' + str(source_system) + '\nsource component: ' + str(source_component)
            if self.logger is not None:
                self.logger.info(s)
            else:
                print(s)

    def connect(self, connection_string=None, baud=None, source_system=255, source_component=0,
                planner_format=None, write=False, append=False, robust_parsing=True, notimestamps=False,
                input=True, dialect='ardupilotmega', autoreconnect=True, zero_time_base=False, use_native=False,
                force_connected=False, progress_callback=None, verbose=True, start_update_thread=True,
                thread_hook_list=[],
                heartbeat_timeout_seconds=10, stream_timeout=20,
                streamrate=5, message_notifier=None, attach_message_hook=True, attach_idle_hook=True):
        """Connect to a mavlink stream.
        Optional arguments which are present in mavutil.mavlink_connection() are transparently passed through to it.

        The object returned by mavutil.mavlink_connection() is referenced in this object's mav field.

        If connection_string is supplied, it will OVERWRITE the connection_string field of this object (normally set by
        its constructor) and then attempt a connection to it.

        Optional arguments other than those of mavutil.mavlink_connection() are as follows:
            verbose (True): log events to this object's logger
            heartbeat_timeout_seconds (10): if this is not set None, this method will wait heartbeat_timeout_seconds for
                a heartbeat to be received on the stream. If it doesn't get one, it will raise TimeoutException.

        TODO: dragons below here
             stream_timeout=20,
                streamrate=5, message_notifier=None, attach_message_hook=True, attach_idle_hook=True)
         # Use ArduPilot dialect and enforce MAVLink2 usage.
         # Set some default streamrate.
         # Add some default utility default hook that serve when receiving messages.
         # This method will try to connect to input arg "connection_string".
         # If no heartbeat is received within "heartbeat_timeout" it will raise a TimeoutException
         # As soon as a heartbeat is received, it will set the Copter object's "target_system" and "target_component"
         # fields to those identified in the mavlink stream.
         # It will then request all data streams ( https://mavlink.io/en/messages/common.html#MAV_DATA_STREAM_ALL ) and
         # set a streamrate. If this has not been achieved within "stream_timeout" it will raise a TimeoutException.
         """

        if self.mavlink2:
            os.environ['MAVLINK20'] = '1'

        self.dialect = dialect
        mavutil.set_dialect(self.dialect)

        if connection_string is not None:
            self.connection_string = connection_string

        if baud is not None:
            self.baud = baud

        if verbose:
            s = 'Attempting connection to: ' + self.connection_string
            if self.logger is not None:
                self.logger.info(s)
            else:
                print(s)

        # Note that this will set self.connection
        self.setup_connection(connection_string=self.connection_string, baud=self.baud, source_system=source_system,
                              source_component=source_component, planner_format=planner_format, write=write,
                              append=append, robust_parsing=robust_parsing, notimestamps=notimestamps, input=input,
                              dialect=dialect, autoreconnect=autoreconnect, zero_time_base=zero_time_base, retries=3,
                              use_native=use_native, force_connected=force_connected,
                              progress_callback=progress_callback, verbose=verbose)

        # We now have set self.connection to the connection so we can pass its reference to this object's mavlink update thread
        self.mavlink_update_thread.connection = self.connection

        # If we are supposed to start the thread, this is an awesome time to do so
        if start_update_thread:
            self.mavlink_update_thread.start(verbose=self.verbose_thread)


        # We intercept heartbeats until we get one from an autopilot ( source component == 1 ) which is
        # running APM ( mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA )
        # if heartbeat_timeout_seconds is not None:
        #     while True:
        #         time_remaining = max(heartbeat_timeout_seconds - (time.time() - tstart), 0)
        #         h = self.mav.wait_heartbeat(blocking=True, timeout=time_remaining)
        #         if h is None:
        #             raise TimeoutException("No heartbeat after " + str(heartbeat_timeout_seconds) + " seconds.")
        #         if h.get_srcComponent() == 1 and h.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        #             break

        # self.target_system = h.get_srcSystem()
        # self.target_component = h.get_srcComponent()

        #
        # if verbose:
        #     self.logger.info('Got a heartbeat:\n' + str(h) + '\nfrom system ' + str(self.target_system) + ', component ' + str(
        #         self.target_component))
        #     self.logger.info('Requesting all data streams')
        #
        # # Ask for all data streams
        # tstart = time.time()
        # while True:
        #     if time.time() - tstart > stream_timeout:
        #         raise TimeoutException("Request for all data streams failed.")
        #     self.mav.mav.request_data_stream_send(
        #         self.target_system,
        #         self.target_component,
        #         mavutil.mavlink.MAV_DATA_STREAM_ALL,
        #         streamrate,
        #         1)
        #     m = self.mav.recv_match(type='SYSTEM_TIME',
        #                             blocking=True,
        #                             timeout=1)
        #     if m is not None:
        #         break
        #
        # self.logger.info('Data stream set, system time is: ' + str(m))
        #
        # if attach_message_hook:
        #     self.mav.message_hooks.append(self.message_hook)
        # if attach_idle_hook:
        #     self.mav.idle_hooks.append(self.idle_hook)
        # if message_notifier is not None:
        #     self.mav.message_hooks.append(message_notifier)











    # def connect_to_mavlink_stream(self, verbose=True, heartbeat_timeout=10, stream_timeout=20, streamrate=5,
    #                               start_update_thread=True, attach_message_hook=False, attach_idle_hook=False,
    #                               baudrate=None, update_thread_verbose=True):
    #     if self.connection_string is None:
    #         raise ValueError(self.name + ': cannot connect because connection_string not set')
    #
    #     # The following line can throw TimeoutException
    #     self.copter.connect(connection_string=self.connection_string, verbose=verbose,
    #                         heartbeat_timeout=heartbeat_timeout, stream_timeout=stream_timeout, streamrate=streamrate,
    #                         message_notifier=self.message_notifier, attach_message_hook=attach_message_hook,
    #                         attach_idle_hook=attach_idle_hook)
    #     self.set_state('CONNECTED')
    #
    #     if start_update_thread:
    #         self.start_mavlink_update_thread(verbose=update_thread_verbose)



if __name__ == "__main__":

    mh = MavlinkHandler(connection_string='udpin:localhost:14550')
    mh.connect(start_update_thread=True, verbose=False)

    # def blip(m):
    #     print(m.get_type())

    # mh.mavlink_update_thread.add_hook(blip)

    # print(1)
    # h = mh.mav.wait_heartbeat()
    # print(2)
    # mh.history.store_message(h)
    # print(3)
    # h = mh.mav.wait_heartbeat()
    # print(4)
    # mh.history.store_message(h)
    #
    # mh.history.source_histories[0].get_message('HEARTBEAT')
    #
    # mh.history.get_last_message('HEARTBEAT')
    # mh.attach_mavlink_update_thread(start_thread=True, hook_list=[blip])
