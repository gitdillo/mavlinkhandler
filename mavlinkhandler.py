from pymavlink import mavutil
import os
import time
import threading

class ErrorException(Exception):
    """Base class for other exceptions"""
    pass

class TimeoutException(ErrorException):
    pass

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
        self.total_received_bytes = 0

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

        self.update_message_stats(msg)

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

    def update_message_stats(self, msg):
        '''
        Reads the passed message's seq and records it in field "last_received_seq", updates total messages received and
        checks if any messages were dropped in between
        '''
        seq = msg.get_seq()
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
        self.total_received_bytes += len(msg.get_msgbuf())

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
        self.record_list = []
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
            # First, check if the message is filtered out
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

    class Record(object):
        '''
        Similar to MessageRequest but instead of returning a single message, keeps a record of multiple messages.
        '''
        def __init__(self, message_type=None, system_id=None, component_id=None):
            self.message_type = message_type
            self.system_id = system_id
            self.component_id = component_id
            self.creation_timestamp = time.time()
            self.message_list = []

        def process_message(self, msg):
            '''
            Checks if the passed message meets the requirements and, if it does, adds it the record
            '''
            # Check if we have set message_type and it clashes
            if self.message_type is not None and self.message_type != msg.get_type():
                return
            # Check if we have set system_id and it clashes
            if self.system_id is not None and self.system_id != msg.get_srcSystem():
                return
            # Check if we have set component_id and it clashes
            if self.component_id is not None and self.component_id != msg.get_srcComponent():
                return

            # Reaching here means the message checks out
            self.message_list.append({'message': msg, 'timestamp': time.time()})

    def add_record(self, message_type=None, system_id=None, component_id=None):
        record = self.Record(message_type=message_type, system_id=system_id, component_id=component_id)
        self.record_list.append(record)
        return record

    def remove_record(self, record):
        if record in self.record_list:
            self.record_list.remove(record)

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

    def add_source_history(self, system_id, component_id):
        '''
        Adds a source history with passed system_id, component_id to this object's "source_histories" field. Returns a
        reference to the added SourceHistory object.
        If a source history with these ids already exists, adds nothing and returns None.
        '''
        # Check that the requested source history does not actually exist
        if self.get_source_history(system_id, component_id) is None:
            if self.verbose_new_messages:
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
            if self.verbose_new_messages:
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

    def store_message(self, msg):
        '''
        Stores the passed message to the appropriate source history. If a source history with the source system and
        component of the message does not exist, it is added to this object's source histories.
        '''
        source_history = self.get_source_history(msg.get_srcSystem(), msg.get_srcComponent())
        if source_history is None:
            source_history = self.add_source_history(msg.get_srcSystem(), msg.get_srcComponent())
        source_history.store_message(msg)
        self.total_messages_received += 1

        # Pass the message by the request list
        for request in self.message_request_list:
            if request.message is None:
                if request.check_message(msg):
                    self.remove_message_request(request)

        # Pass the message by the record list
        for record in self.record_list:
            record.process_message(msg)

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

    def __init__(self, connection, logger=None, name='MavlinkUpdateThread', hook_list=[], update_loop_period_sec=0.01):

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
                             kwargs={'message_timeout_sec': 0.5,
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

    def update(self, message_timeout_sec=5, verbose=True):

        while True:

            t_start = time.time()

            if self.termflag.is_set():
                s = str(self.name) + ': MAVLINK update thread exiting.'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
                self.isPaused = False
                self.isRunning = False
                return

            if self.pauseflag.is_set():
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

            # Let's try to grab a message
            msg = None
            try:
                msg = self.connection.recv_match(blocking=True, timeout=message_timeout_sec)
            except Exception as e:
                if verbose:
                    s = str(self.name) + ':  while listening for incoming messages got exception: ' + str(e)
                    if self.logger is not None:
                        self.logger.info(s)
                    else:
                        print(s)
                continue
            if msg is None:
                if verbose:
                    s = str(self.name) + ': ' + str(
                        message_timeout_sec) + ' second timeout while waiting for MAVLINK message'
                    if self.logger is not None:
                        self.logger.info(s)
                    else:
                        print(s)
            else:
                for function in self.hook_list:
                    function(msg)

            sleep_time = self.update_loop_period_sec - (time.time() - t_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

class MavlinkHandler(object):

    def __init__(self, connection_string='udpin:0.0.0.0:14550', baud=57600, mavlink2=True, dialect='ardupilotmega',
                 logger=None, name='MavlinkHandler', thread_name=None, thread_logger=None, thread_start=False,
                 mavlink_history_depth=10, verbose_thread=False, verbose_message_drop=False, verbose_new_messages=True):

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
                                      verbose_message_drop=verbose_message_drop,
                                      verbose_new_messages=verbose_new_messages)

        # NOTE: we add this object's history's store_message method as an initial hook to the mavlink update thread
        self.attach_mavlink_update_thread(name=thread_name, hook_list=[self.history.store_message],
                                          start_thread=thread_start, logger=thread_logger, verbose=self.verbose_thread)

        # Expose some of the thread's methods through the handler
        self.add_hook = self.mavlink_update_thread.add_hook
        self.remove_hook = self.mavlink_update_thread.remove_hook

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

        self.mavlink_update_thread = MavlinkUpdateThread(self.connection, logger=logger, name=name, hook_list=hook_list)
        if start_thread:
            self.mavlink_update_thread.start(verbose=verbose)

    def set_system_id(self, system_id):
        '''
        If connection (field .mav of this mav) has been initialised (via this object's connect() method), sets its
        .srcSystem field to the passed id
        '''
        if self.connection is not None:
            self.connection.mav.srcSystem = system_id

    def set_component_id(self, component_id):
        '''
        If connection (field .mav of this mav) has been initialised (via this object's connect() method), sets its
        .srcComponent fields to the passed id
        '''
        if self.connection is not None:
            self.connection.mav.srcComponent = component_id

    def get_system_id(self):
        '''
        If connection (field .mav of this mav) has been initialised (via this object's connect() method), return its
        .srcSystem field
        '''
        if self.connection is not None:
            return self.connection.mav.srcSystem

    def get_component_id(self):
        '''
        If connection (field .mav of this mav) has been initialised (via this object's connect() method), return its
        .srcComponent field
        '''
        if self.connection is not None:
            return self.connection.mav.srcComponent

    def send_message(self, message, preserve_source=False, verbose=True,
                     force_mavlink1=False, wait_altered_source_clear_seconds=1, wait_loop_period_seconds=0.001):
        '''
        Sends a mavlink message, assuming a connection has been set up (field "connection" is not None).

        If preserve_source is set to True, the passed message will be transmitted as is, i.e. using what exists in its
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

        if self.get_system_id() is None:
            if verbose:
                s = str(self.name) + ': cannot send since system id has not yet been set'
                if self.logger is not None:
                    self.logger.info(s)
                else:
                    print(s)
            return

        if self.get_component_id() is None:
            if verbose:
                s = str(self.name) + ': cannot send since component id has not yet been set'
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
                original_system_id = self.get_system_id()
                original_component_id = self.get_component_id()
                self.altered_source = True      # signal that we are transmitting with different source ids
                self.set_system_id(message.get_srcSystem())
                self.set_component_id(message.get_srcComponent())
                self.connection.mav.send(message)
                self.connection.mav.seq = (self.connection.mav.seq - 1) % 256   # restore the seq incremented by self.connection.mav.seq
                self.set_system_id(original_system_id)
                self.set_component_id(original_component_id)
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

    def send_get_response(self, message, response_message_type, timeout_sec=5, wait_loop_period_sec=0.01):
        '''
        Sends a message and grabs some expected response. This is very useful for all interactions where some specific
        response is expected from a component. This is a blocking command which can raise TimeoutException.

        Inputs:
            message: the message to be sent out (pymavlink message object)
            response_message_type: the type (str) we expect to get back (get_type() of pymavlink message object)
            timeout_sec=5: timeout. If it is reached, this method will raise TimeoutException
            wait_loop_period_sec=0.01: time the internal loop sleeps for between checks for valid response

        Returns:
            Either the response (pymavlink message object) or raises TimeoutException if no response is intercepted
            within its timeout.
        '''

        # Start a record, mark the time and send the message
        record = self.history.add_record(response_message_type)
        t0 = time.time()
        self.send_message(message)

        # Twiddle thumbs till we hear back or timeout
        while True:
            if time.time() - t0 > timeout_sec:
                self.history.remove_record(record)  # clean up the record before exiting
                raise TimeoutException('Timeout while waiting for response of type: ' + str(response_message_type))

            # presumably, some good stuff will eventually appear in record.message_list, let's check
            for r in record.message_list:
                response = r['message']
                timestamp = r['timestamp']

                # Filter out anything we don't like
                if not response.get_type() == response_message_type: # this shouldn't ever happen but let's check anyway
                    continue

                if timestamp < t0:  # received response arrived before we sent out our message
                    continue

                if 'target_system' in message.fieldnames:       # outgoing message had a target system...
                    if not  message.target_system == response.get_srcSystem():   # response from wrong system
                        continue

                if 'target_component' in message.fieldnames:    # outgoing message had a target component...
                    if not message.target_component == response.get_srcComponent():   # response from wrong system
                        continue

                # Reaching here means we got a response that survived the filtering. Return it.
                self.history.remove_record(record)  # clean up the record before exiting
                return response

            time.sleep(wait_loop_period_sec)

    def _setup_connection(self, connection_string, baud, source_system, source_component,
                          planner_format, write, append, robust_parsing, notimestamps,
                          input, dialect, autoreconnect, zero_time_base, retries,
                          use_native, force_connected, progress_callback, verbose):
        '''
        Do not use directly, rather use connect() which calls this one.
        '''

        if connection_string is not None:
            if verbose:
                s = str(self.name) + ': _setup_connection: setting connection string to ' + str(connection_string)
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


    def connect(self, connection_string=None, baud=None, source_system=None, source_component=None,
                planner_format=None, write=False, append=False, robust_parsing=True, notimestamps=False,
                input=True, dialect='ardupilotmega', autoreconnect=True, zero_time_base=False, retries=3,
                use_native=False, force_connected=False, progress_callback=None, verbose=True, start_update_thread=True,
                thread_hook_list=[]):
        """Connect to a mavlink stream.
        Optional arguments which are present in mavutil.mavlink_connection() are transparently passed through to it.

        The object returned by mavutil.mavlink_connection() is referenced in this object's mav field.

        If connection_string is supplied, it will OVERWRITE the connection_string field of this object (normally set by
        its constructor) and then attempt a connection to it.

        Optional arguments other than those of mavutil.mavlink_connection() are as follows:
            verbose (True): log events to this object's logger
            heartbeat_timeout_seconds (10): if this is not set None, this method will wait heartbeat_timeout_seconds for
                a heartbeat to be received on the stream. If it doesn't get one, it will raise TimeoutException.
         """

        # Massage the inputs into reasonable values

        # Connection string is definitely needed so let's hope we got it passed now or got it through the constructor
        if connection_string is not None:
            self.connection_string = connection_string
        elif self.connection_string is None:
            raise ValueError('No connection provided to connect() or set in the constructor of MavlinkHandler')

        # System and component id are not necessary but if not passed, will be set to 0 by mavutil.mavlink_connection()
        # Warn the user of potential for idiocy irrespective of verbosity
        missing=[]
        if source_system is None:
            missing.append('source_system')
        if source_component is None:
            missing.append('source_component')
        if missing:
            s = '*** WARNING: ' + str(self.name) + ': Not provided: '
            s += missing[0]
            # Try our luck adding a second missing item
            try:
                s += ', ' + missing[1]
            except IndexError:
                pass
            s += '. Will set to 0 but this is bad practice. Set to correct values to avoid seeing this message. ***'
            if self.logger is not None:
                self.logger.info(s)
            else:
                print(s)




        # For all other args, we can use defaults and hope for the best
        if baud is not None:
            self.baud = baud

        if self.mavlink2:
            os.environ['MAVLINK20'] = '1'

        self.dialect = dialect
        mavutil.set_dialect(self.dialect)

        if verbose:
            s = 'Attempting connection to: ' + self.connection_string
            if self.logger is not None:
                self.logger.info(s)
            else:
                print(s)

        self._setup_connection(
            connection_string=self.connection_string,
            baud=self.baud,
            source_system=source_system,
            source_component=source_component,
            planner_format=planner_format,
            write=write,
            append=append,
            robust_parsing=robust_parsing,
            notimestamps=notimestamps,
            input=input,
            dialect=dialect,
            autoreconnect=autoreconnect,
            zero_time_base=zero_time_base,
            retries=retries,
            use_native=use_native,
            force_connected=force_connected,
            progress_callback=progress_callback,
            verbose=verbose
        )

        # We now have set self.connection to the connection so we can pass its reference to this object's mavlink update thread
        self.mavlink_update_thread.connection = self.connection

        # If we are supposed to start the thread, this is an awesome time to do so
        if start_update_thread:
            for hook in thread_hook_list:                   # add any passed hooks before starting the thread
                self.mavlink_update_thread.add_hook(hook)
            self.mavlink_update_thread.start(verbose=self.verbose_thread)


if __name__ == "__main__":
    # If called as script, try to connect to UDP 14550
    mh = MavlinkHandler(connection_string='udpin:localhost:14550')
    mh.connect(start_update_thread=True, verbose=False)