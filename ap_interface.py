from mavlinkhandler import mavlinkhandler
from pymavlink import mavutil
import time
import logging

class AutopilotInterface:
    '''
    Class for managing a connection to an autopilot whose messages are hopefully present in a mavlink stream.

    Provides methods for interacting to an autopilot, likely to be communicating at the very minimum using the
    MAVLINK Common Message Set:
    https://mavlink.io/en/messages/common.html#mavlink-common-message-set

    NOTE: component id can be set here but we will assume the system id of the detected autopilot once we receive an
    appropriate message.
    '''

    def __init__(self, connection_string, connection_baud=None, autopilot_sysid=None, MAV_TYPE=18, MAV_AUTOPILOT=8,
                 MAV_STATE=3, component_id=191, autospawn_mavlink_handler=True, logger=None, verbose_connection=True,
                 verbose_new_messages=True):

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
        self.system_id = None   # this is set when we receive an FCU heartbeat

        # The following will be auto filled when we connect to an FCU
        self.mavlink_handler = None
        self.connected_fcu_sysid = None
        self.connected_fcu_compid = None
        self.connected_fcu_history = None

        # Convenience field, possibly add an externally supplied logger
        self.logger = logger

        # Chattiness level modifiers
        self._verbose_connection = verbose_connection
        self._verbose_new_messages = verbose_new_messages


        if autospawn_mavlink_handler:
            self.spawn_mavlink_handler(verbose=self._verbose_connection)

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
        NOTE: when the spawned mavlink handler's thread detects an incoming message from an FCU, it will automatically
        set self.connected_fcu_sysid (defaults to None when object created).
        This is a possible way to test if an FCU has been detected on the stream.
        '''

        def FCU_connection_setup_hook(msg):
            '''
            This hook will be added to the mavlink handler by this method. It will monitor the history until it sees a
            message from a source with component id 1 (identifying as an autopilot).
            It will then assume this is the FCU we connect to which will give us our own system id (we want the same
            sysid as the FCU) which we set in self.system_id.
            It will set this object's fields connected_fcu_sysid and connected_fcu_compid
            It will set this object's field connected_fcu_history to the mavlink handler's history assigned to the FCU
            It will then remove itself from the hook list.
            '''
            # Check if we have received a message from a source with component id 1 (identifies as autopilot)
            for source in self.mavlink_handler.history.get_source_ids():
                if source['component'] == 1:    # Bingo, this source claims to be an FCU

                    # Just one final thing to check, if we have been given a target autopilot sysid, we need to check
                    # this is the correct one
                    if self._target_autopilot_sysid is not None and self._target_autopilot_sysid == source['system']:
                        continue

                    self.system_id = source['system']

                    # Set this object's _connected_fcu... fields
                    self.connected_fcu_sysid = source['system']
                    self.connected_fcu_compid = source['component']

                    # Ensure this object's and its mavlink handler are using the same ids
                    self.mavlink_handler.set_system_id(self.system_id)
                    self.mavlink_handler.set_component_id(self.component_id)

                    # Redirect this object's sysids to those of its mavlink handler
                    self.system_id = self.mavlink_handler.get_system_id()
                    self.component_id = self.mavlink_handler.get_component_id()

                    # Set field connected_fcu_history to the history of the connected FCU
                    self.connected_fcu_history = self.mavlink_handler.history.get_source_history(
                        self.connected_fcu_sysid, self.connected_fcu_compid)

                    # Fire off a heartbeat to let the other side we are here
                    if verbose:     # note that this is the "verbose" variable of the enclosing method
                        self.log('Connected to an autopilot with system:component IDs: ' + str(
                            self.connected_fcu_sysid) + ':' + str(
                            self.connected_fcu_compid) + '. Setting own IDs to: ' + str(self.system_id) + ':' + str(
                            self.component_id) + ' and responding with a heartbeat')
                    self.send_heartbeat()

                    # Finally, remove this method from the hook list
                    self.mavlink_handler.remove_hook(FCU_connection_setup_hook)

        self.mavlink_handler = mavlinkhandler.MavlinkHandler(logger=self.logger,
                                                             verbose_new_messages=self._verbose_new_messages)
        self.mavlink_handler.add_hook(FCU_connection_setup_hook)    # this hook will set our sysid when it encounters an FCU and will then remove itself
        self.mavlink_handler.connect(connection_string=self.connection_string, baud=self.baud, source_system=0,
                                     source_component=self.component_id, start_update_thread=True,
                                     verbose=self._verbose_connection)


    def request_message_interval(self, message_type, interval_us, confirmation=0, response_target=1, blocking=True,
                                 timeout_sec=5):
        '''
        Sends a MAV_CMD_SET_MESSAGE_INTERVAL to the FCU for a message identified by its type. For messages see:
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

        cl = self.mavlink_handler.connection.mav.command_long_encode(self.connected_fcu_sysid,
                                                     self.connected_fcu_compid,
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
            if message_type in self.connected_fcu_history.get_message_type_list():
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

        self.mavlink_handler.connection.mav.command_long_send(self.connected_fcu_sysid, self.connected_fcu_compid, 246,
                                                            0, 1, 0, 0, 0, 0, 0, 0)

    def send_tune_to_FCU(self, tune_string, logger=None):
        try:
            self.mavlink_handler.connection.mav.play_tune_send(self.connected_fcu_sysid, self.connected_fcu_compid,
                                                               bytes(tune_string, 'ascii'))
        except Exception as e:
            s = 'While trying to send tune to FCU, the following exception occurred: ' + str(e)
            s += ' , WIRE_PROTOCOL_VERSION: ' + self.mavlink_handler.connection.WIRE_PROTOCOL_VERSION
            if logger is None:
                print(s)
            else:
                logger.error(s)

    def send_success_tune_to_FCU(self, logger=None):
        self.send_tune_to_FCU('G8B', logger=logger)

    def send_error_tune_to_FCU(self, logger=None):
        # 'BGBG' also possible but a bit too harsh
        self.send_tune_to_FCU('E8C', logger=logger)

    def send_script_off_tune_to_FCU(self, logger=None):
        self.send_tune_to_FCU('E16D16C', logger=logger)

    def send_companion_poweroff_tune_to_FCU(self, logger=None):
        self.send_tune_to_FCU('B8A8G8F8E8D8C1', logger=logger)

    def send_progress_bip_to_FCU(self, logger=None):
        self.send_tune_to_FCU('G16', logger=logger)

    def send_test_start_tune_to_FCU(self, logger=None):
        self.send_tune_to_FCU('C8D8E8')

    def send_test_end_tune_to_FCU(self, logger=None):
        self.send_tune_to_FCU('E8D8C8')




