from pymavlink import mavutil
import time
import logging

# The following is awful but preferable to spending my life getting import to stop being retarded
try:
    from mavlinkhandler.mavlinkhandler import MavlinkHandler
except ImportError:
    from mavlinkhandler import MavlinkHandler



class AutopilotInterface:
    '''
    Class for managing a connection to an autopilot whose messages are hopefully present in a mavlink stream.

    Provides methods for interacting to an autopilot, likely to be communicating at the very minimum using the
    MAVLINK Common Message Set:
    https://mavlink.io/en/messages/common.html#mavlink-common-message-set

    Information on system and component IDs:
    https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html#mavlink-routing-in-ardupilot

    NOTE: the autopilot will have its own system:component ID pair. In the case of an onboard computer we ***assume***
    that we want our own system id to reflect that autopilot's since all physical components on a vehicle should share
    the same system id (this is the default assumption for this software). However, we might want to have a user defined
    system id, for example, if we are a Ground Control Station. For more info, see the constructor documentation.
    '''

    def __init__(self, connection_string, connection_baud=None, autopilot_sysid=None, own_sysid=None, MAV_TYPE=18,
                 MAV_AUTOPILOT=8, MAV_STATE=3, component_id=191, autospawn_mavlink_handler=True, logger=None,
                 verbose_connection=True, verbose_new_messages=True, connection_timeout_seconds=None):
        '''
        Creates an Autopilot_interface, a class encapsulating a Mavlink Handler, but geared specifically towards
        interactions with an autopilot as opposed to any arbitrary mavlink components whose messages might be present in
        a mavlink data stream.

        TODO: add info on fields.

        :param connection_string: connection string, see https://mavlink.io/en/mavgen_python/#connection_string
        :param connection_baud: set this if using serial connection strings, ignored otherwise
        :param autopilot_sysid: leave None to connect to the first identified autopilot. If set, will only connect to
            the autopilot with the specified system id.
        :param own_sysid: leave None to be assigned the system id of the autopilot. If set, the set value will be used.
        :param MAV_TYPE: our own MAV_TYPE (https://mavlink.io/en/messages/common.html#MAV_TYPE). Default 18 (Onboard
            companion controller)
        :param MAV_AUTOPILOT: our own MAV_AUTOPILOT type (https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT).
            Default 8 (invalid).
        :param MAV_STATE: our own MAV_STATE (https://mavlink.io/en/messages/common.html#MAV_STATE). Default 3 (standby).
        :param component_id: our own component id (https://mavlink.io/en/messages/common.html#MAV_COMPONENT). Default\
            191 (companion computer).
        :param autospawn_mavlink_handler: default True, to spawn a mavlink handler listening for incoming messages at
            object creation. Set to False if needed to do it manually (e.g. debugging).
        :param logger: Optionally pass a logger (from package logging) for output to be directed to. If no logger is
            passed, output is printed.
        :param verbose_connection: set True to print connection related messages.
        :param verbose_new_messages: set True to print a message each time novel mavlink message type is encountered.
        :param connection_timeout_seconds: if set to a non None value, will raise a TimeoutError if no autopilot has
            been identified by the time of the timeout. Setting it to None will return an object immediately but this
            object might not have had connected to an autopilot yet. To test for this, look at the "autopilot_connected"
            field which will be True once a suitable (in case we have set "autopilot_sysid" above) autopilot has been
            identified.
        '''

        # The following are what we really care about, connect there and listen for these messages
        self.connection_string = connection_string
        self.baud = connection_baud = connection_baud

        # If this is not None, we will only consider a valid autopilot an autopilot that broadcasts this sysid
        self._target_autopilot_sysid = autopilot_sysid

        # The following are for our own identification via heartbeat, see:
        # https://mavlink.io/en/messages/common.html#HEARTBEAT
        self.MAV_TYPE = MAV_TYPE
        self.MAV_AUTOPILOT = MAV_AUTOPILOT
        self.MAV_STATE = MAV_STATE
        self.component_id = component_id
        self.system_id = own_sysid   # if this is None, it is set when we receive an autopilot heartbeat

        # The following will be auto filled when we connect to an autopilot
        self.autopilot_connected = False
        self.mavlink_handler = None
        self.connected_autopilot_sysid = None
        self.connected_autopilot_compid = None
        self.connected_autopilot_history = None

        # Convenience field, possibly add an externally supplied logger
        self.logger = logger

        # Chattiness level modifiers
        self._verbose_connection = verbose_connection
        self._verbose_new_messages = verbose_new_messages


        if autospawn_mavlink_handler:
            self.spawn_mavlink_handler(verbose=self._verbose_connection)

            # If we have been given a non None connection timeout, wait until we get a connection
            if connection_timeout_seconds is not None:
                t0 = time.time()
                while True:
                    delta_t = time.time() - t0
                    if delta_t > connection_timeout_seconds:
                        raise TimeoutError(
                            'Timeout waiting after waiting ' + str(delta_t) + ' seconds for an autopilot connection')
                    time.sleep(0.1)
                    if self.autopilot_connected:
                        return


    def log(self, message, level=logging.INFO):
        '''
        If field "logger" has been initialised, it logs the message there (at optional passed log level), otherwise
        it prints it.
        '''
        if self.logger is None:
            print(message)
        else:
            self.logger.log(level, message)


    def spawn_mavlink_handler(self, verbose=True):
        '''
        NOTE: when the spawned mavlink handler's thread detects an incoming message from an autopilot, it will
        automatically set self.connected_autopilot_sysid (defaults to None when object created).
        This is a possible way to test if an autopilot has been detected on the stream.
        '''

        def autopilot_connection_setup_hook(msg):
            '''
            This hook will be added to the mavlink handler by this method. It will monitor the history until it sees a
            message from a source with component id mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 (this means a component
            identifying as an autopilot).

            It will then assume this is the autopilot we connect to which will give us our own system id (we want the
                same sysid as the autopilot) which we set in self.system_id.
            NOTE: if we have passed "autopilot_sysid" to the constructor of "AutopilotInterface", we will only accept a
                connection to an autopilot with that sysid!

            It will set this object's fields connected_autopilot_sysid and connected_autopilot_compid
            It will set this object's field connected_autopilot_history to the mavlink handler's history assigned to the
                autopilot.
            It will then remove itself from the hook list.
            '''
            # Check if we have received a message from a source with component id 1 (identifies as autopilot)
            for source in self.mavlink_handler.history.get_source_ids():
                if source['component'] == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:    # Bingo, this source claims to be an autopilot

                    # Just one final thing to check, if we have been given a target autopilot sysid, we need to check
                    # this is the correct one
                    if self._target_autopilot_sysid is not None and self._target_autopilot_sysid != source['system']:
                        continue

                    # If we have not been given a system id, we assign that of the detected autopilot
                    if self.system_id is None:
                        self.system_id = source['system']

                    # Set this object's connected_autopilot... fields
                    self.connected_autopilot_sysid = source['system']
                    self.connected_autopilot_compid = source['component']

                    # Ensure this object's and its mavlink handler are using the same ids
                    self.mavlink_handler.set_system_id(self.system_id)
                    self.mavlink_handler.set_component_id(self.component_id)

                    # Redirect this object's sysids to those of its mavlink handler
                    self.system_id = self.mavlink_handler.get_system_id()
                    self.component_id = self.mavlink_handler.get_component_id()

                    # Set field connected_autopilot_history to the history of the connected FCU
                    self.connected_autopilot_history = self.mavlink_handler.history.get_source_history(
                        self.connected_autopilot_sysid, self.connected_autopilot_compid)

                    # Raise the autopilot connected flag
                    self.autopilot_connected = True

                    # Fire off a heartbeat to let the other side we are here
                    if verbose:     # note that this is the "verbose" variable of the enclosing method
                        self.log('Connected to an autopilot with system:component IDs: ' + str(
                            self.connected_autopilot_sysid) + ':' + str(
                            self.connected_autopilot_compid) + '. Setting own IDs to: ' + str(self.system_id) + ':' + str(
                            self.component_id) + ' and responding with a heartbeat.')
                    self.send_heartbeat()

                    # Finally, remove this method from the hook list
                    self.mavlink_handler.remove_hook(autopilot_connection_setup_hook)

        self.mavlink_handler = MavlinkHandler(logger=self.logger,
                                                             verbose_new_messages=self._verbose_new_messages)
        self.mavlink_handler.add_hook(autopilot_connection_setup_hook)    # this hook will set our sysid when it encounters an autopilot and will then remove itself
        self.mavlink_handler.connect(connection_string=self.connection_string, baud=self.baud, source_system=0,
                                     source_component=self.component_id, start_update_thread=True,
                                     verbose=self._verbose_connection)


    def run_command_long(self, command, params, expected_result=mavutil.mavlink.MAV_RESULT_ACCEPTED, timeout_sec=5.0,
                         confirmation=0, verbose=True):
        '''
        Sends a command long and returns its result.

        Inputs:
            command: (MAV_CMD) the numberic id of the command to send,
                see: https://mavlink.io/en/messages/common.html#mav_commands

            params: the params for the command. This has to be a list with 7 elements.

            expected_result: the numeric value of the MAV_RESULT from the expected COMMAND_ACK,
                see: https://mavlink.io/en/messages/common.html#MAV_RESULT
                see: https://mavlink.io/en/messages/common.html#COMMAND_ACK

            timeout_sec: the timeout for this blocking command in seconds.

            confirmation: confirmation field of COMMAND_LONG, normally leave to default 0,
                see: https://mavlink.io/en/messages/common.html#COMMAND_LONG

        Returns:
            A dict of the form: {'success': False, 'ack': ack}
                "success" will be True if we got a COMMAND_ACK with "expected_result" (see inputs).
                If the result is not "expected_result", "success" will be False
                "ack" is the returned COMMAND_ACK (pymavlink message object)

            If no COMMAND_ACK has been received within "timeout_sec", it will raise TimeoutException
        '''
        if not len(params) == 7:
            raise TypeError('params arg has to be a list of 7 items, '
                            'see:\nhttps://mavlink.io/en/messages/common.html#COMMAND_LONG')

        if self.connected_autopilot_sysid is None:
            raise ValueError('connected_autopilot_sysid not set yet')

        if self.connected_autopilot_compid is None:
            raise ValueError('connected_autopilot_compid not set yet')

        # First, let's construct our command
        cmd = self.mavlink_handler.connection.mav.command_long_encode(self.connected_autopilot_sysid,
                                                                      self.connected_autopilot_compid,
                                                                      command,
                                                                      confirmation,
                                                                      params[0],
                                                                      params[1],
                                                                      params[2],
                                                                      params[3],
                                                                      params[4],
                                                                      params[5],
                                                                      params[6],
                                                                      )

        # Now send and grab response, while marking the time
        t0=time.time()
        # This can throw TimeoutException if no response within timeout_sec. We don't catch it, let it propagate up
        ack = self.mavlink_handler.send_get_response(cmd, 'COMMAND_ACK', timeout_sec=timeout_sec)
        dt = time.time() - t0

        if ack.result == expected_result:
            if verbose:
                s = ('%s: got successful ACK: %s in %.4f seconds' % (self.mavlink_handler.name, mavutil.mavlink.enums["MAV_RESULT"][ack.result].name, dt))
                self.log(s, level=logging.INFO)
            return {'success': True, 'ack': ack}
        else:
            error_string = "%s: Expected %s got %s\nDescription:\n%s" % (
                self.autopilot_mavlink_handler.name,
                mavutil.mavlink.enums["MAV_RESULT"][expected_result].name,
                mavutil.mavlink.enums["MAV_RESULT"][ack.result].name,
                mavutil.mavlink.enums["MAV_RESULT"][ack.result].description
            )
            if verbose:
                self.log(error_string, level=logging.INFO)
            return {'success': False, 'ack': ack}



    def request_message_interval(self, message_type, interval_us, confirmation=0, response_target=1, blocking=True,
                                 timeout_sec=5):
        '''
        Sends a MAV_CMD_SET_MESSAGE_INTERVAL to the autopilot for a message identified by its type. For messages see:
        https://mavlink.io/en/messages/common.html
        and
        https://mavlink.io/en/messages/ardupilotmega.html

        Raises ValueError if the message type cannot be resolved to a message id (resolution uses this object's
            get_message_id_by_type() method).

        If blocking=True, will wait until timeout_sec to see the requested message appear in the stream
        '''
        # To understand the passed values, see COMMAND_LONG at:
        # https://mavlink.io/en/messages/common.html#COMMAND_LONG
        # and MAV_CMD_SET_MESSAGE_INTERVAL at:
        # https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

        message_id = self.get_message_id_by_type(message_type)

        if message_id is None:
            raise ValueError('Unknown message type: ' + str(message_type))

        t0=time.time()

        cl = self.mavlink_handler.connection.mav.command_long_encode(self.connected_autopilot_sysid,
                                                                     self.connected_autopilot_compid,
                                                                     511,  # == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
                                                                     confirmation,
                                                                     message_id,
                                                                     interval_us,
                                                                     0, 0, 0, 0,
                                                                     response_target)

        self.mavlink_handler.connection.mav.send(cl)

        time.sleep(0.1)
        while True:
            if time.time() - t0 > timeout_sec:
                return False
            if message_type in self.connected_autopilot_history.get_message_type_list():
                return True

    def get_message_id_by_type(self, msg_type):
        try:
            return [mavutil.mavlink.mavlink_map[a].id for a in mavutil.mavlink.mavlink_map if
                    mavutil.mavlink.mavlink_map[a].msgname == msg_type][0]
        except KeyError:
            return None
        except IndexError:
            return None

    def get_message_type_by_id(self, msg_id):
        try:
            return mavutil.mavlink.mavlink_map[msg_id].msgname
        except KeyError:
            return None
        except IndexError:
            return None

    def send_heartbeat(self):
        self.mavlink_handler.connection.mav.heartbeat_send(self.MAV_TYPE, self.MAV_AUTOPILOT, 0, 0, self.MAV_STATE)

    def reboot_autopilot(self, i_really_mean_it=False):
        '''
        For testing purposes only. Be very, very careful
        '''
        if i_really_mean_it is False:
            return

        self.mavlink_handler.connection.mav.command_long_send(self.connected_autopilot_sysid, self.connected_autopilot_compid, 246,
                                                              0, 1, 0, 0, 0, 0, 0, 0)

    def send_tune_to_autopilot(self, tune_string, logger=None):
        try:
            self.mavlink_handler.connection.mav.play_tune_send(self.connected_autopilot_sysid, self.connected_autopilot_compid,
                                                               bytes(tune_string, 'ascii'))
        except Exception as e:
            s = 'While trying to send tune to autopilot, the following exception occurred: ' + str(e)
            s += ' , WIRE_PROTOCOL_VERSION: ' + self.mavlink_handler.connection.WIRE_PROTOCOL_VERSION
            if logger is None:
                print(s)
            else:
                logger.error(s)

    # def send_success_tune_to_FCU(self, logger=None):
    #     self.send_tune_to_autopilot('G8B', logger=logger)
    #
    # def send_error_tune_to_FCU(self, logger=None):
    #     # 'BGBG' also possible but a bit too harsh
    #     self.send_tune_to_autopilot('E8C', logger=logger)
    #
    # def send_script_off_tune_to_FCU(self, logger=None):
    #     self.send_tune_to_autopilot('E16D16C', logger=logger)
    #
    # def send_companion_poweroff_tune_to_FCU(self, logger=None):
    #     self.send_tune_to_autopilot('B8A8G8F8E8D8C1', logger=logger)
    #
    # def send_progress_bip_to_FCU(self, logger=None):
    #     self.send_tune_to_autopilot('G16', logger=logger)
    #
    # def send_test_start_tune_to_FCU(self, logger=None):
    #     self.send_tune_to_autopilot('C8D8E8')
    #
    # def send_test_end_tune_to_FCU(self, logger=None):
    #     self.send_tune_to_autopilot('E8D8C8')




