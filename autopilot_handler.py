from pymavlink import mavutil
import time
from mavlinkhandler import MavlinkHandler, TimeoutException

class AutopilotHandler(object):
    '''
    A simple class monitoring a mavlink stream from an autopilot, updating some internal fields and offering an API for
    some commands.

    Typically, this would run in a companion computer but might also run in a GCS.
    '''
    def __init__(self, autopilot_connection_string, system_id=None, component_id=None):
        self.autopilot_mavlink_handler = MavlinkHandler(name='autopilot_mavlink_handler')   # name is optional, for making error messages more understandable
        self.autopilot_mavlink_handler.mavlink_update_thread.add_hook(self.update_fields)
        self.autopilot_mavlink_handler.mavlink_update_thread.add_hook(self.process_status_text)

        self.autopilot_mavlink_handler.connect(connection_string=autopilot_connection_string, start_update_thread=True,
                                               source_system=system_id, source_component=component_id)

        # Stuff we will read off the autopilot once we hear from it
        self.autopilot_system_id = None     # System ID, set by the autopilot onboard the vehicle
        self.autopilot_component_id = None  # https://mavlink.io/en/messages/common.html#MAV_COMPONENT should be 1 for autopilot
        self.autopilot_class = None         # https://mavlink.io/en/messages/common.html#MAV_AUTOPILOT

        # Fields below are automatically updated in update_fields() by our mavlink_update_thread
        self.armed = None
        self.mode = None
        self.mav_state = None
        self.GPS_unix_time_us = None
        self.groundspeed = None
        self.airspeed = None
        self.throttle_percent = None
        self.latitude = None
        self.longitude = None
        self.altitude_abs = None
        self.altitude_rel = None
        self.compass_heading = None
        self.battery_voltages = None
        self.battery_current = None
        self.battery_remaining_percentage = None
        self.battery_current_consumed_mAh = None
        self.battery_energy_consumed_hJ = None
        self.battery_time_remaining_sec = None
        self.battery_charge_state = None
        self.current_mission_item = None
        self.current_mission_item_timestamp = None
        self.last_reached_mission_item = None
        self._last_reached_mission_item_timestamp = None
        self.target_location_latitude = None
        self.target_location_longitude = None
        self.target_location_alt_abs = None
        self.GPS_status_satellites_visible = None
        self.GPS_status_time_usec = None
        self.GPS_status_fix_type = None
        self.GPS_status_fix_type_description = None

    def update_fields(self, message):
        '''
        Updates this object's fields according to received messages
        '''
        # Filter out stuff not coming from the autopilot
        if not message.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
            return

        # HEARTBEAT gives arm state, flight mode and mav state
        if message.get_type() == 'HEARTBEAT':
            # Arm state first
            if (message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == 0:
                self.armed = False
            else:
                self.armed = True
            # Mode: we translate using the mavutil.mode_string_v10() method
            self.mode = mavutil.mode_string_v10(message)
            # MAV_STATE: see https://mavlink.io/en/messages/common.html#MAV_STATE
            # also mapping in mavutil.mavlink.MAV_STATE_XXX
            old_state = self.mav_state
            if message.system_status == mavutil.mavlink.MAV_STATE_UNINIT:
                self.mav_state = 'MAV_STATE_UNINIT'
            elif message.system_status == mavutil.mavlink.MAV_STATE_BOOT:
                self.mav_state = 'MAV_STATE_BOOT'
            elif message.system_status == mavutil.mavlink.MAV_STATE_CALIBRATING:
                self.mav_state = 'MAV_STATE_CALIBRATING'
            elif message.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
                self.mav_state = 'MAV_STATE_STANDBY'
            elif message.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
                self.mav_state = 'MAV_STATE_ACTIVE'
            elif message.system_status == mavutil.mavlink.MAV_STATE_CRITICAL:
                self.mav_state = 'MAV_STATE_CRITICAL'
            elif message.system_status == mavutil.mavlink.MAV_STATE_EMERGENCY:
                self.mav_state = 'MAV_STATE_EMERGENCY'
            elif message.system_status == mavutil.mavlink.MAV_STATE_POWEROFF:
                self.mav_state = 'MAV_STATE_POWEROFF'
            elif message.system_status == mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION:
                self.mav_state = 'MAV_STATE_FLIGHT_TERMINATION'
            else:
                self.mav_state = 'MAV_STATE_UNINIT'
            if not old_state == self.mav_state:
                print('State changed to: ' + str(self.mav_state))
            return

        # SYSTEM_TIME gives GPS time
        if message.get_type() == 'SYSTEM_TIME':
            self.GPS_unix_time_us = message.time_unix_usec
            return

        # VFR_HUD gives us airspeed, groundspeed and throttle percentage (among others)
        if message.get_type() == 'VFR_HUD':
            self.groundspeed = message.groundspeed
            self.airspeed = message.airspeed
            self.throttle_percent = message.throttle
            return

        # GLOBAL_POSITION_INT gives position
        if message.get_type() == 'GLOBAL_POSITION_INT':
            self.latitude = message.lat * 1e-7
            self.longitude = message.lon * 1e-7
            self.altitude_abs = message.alt * 1e-3
            self.altitude_rel = message.relative_alt * 1e-3
            self.compass_heading = message.hdg * 1e-2
            return

        # BATTERY_STATUS tells us about the battery
        if message.get_type() == 'BATTERY_STATUS':
            self.battery_voltages = message.voltages
            self.battery_current = message.current_battery * 1e-2
            self.battery_remaining_percentage = message.battery_remaining
            self.battery_current_consumed_mAh = message.current_consumed
            self.battery_energy_consumed_hJ = message.energy_consumed
            self.battery_time_remaining_sec = message.time_remaining
            # Enums for charge state are defined here: https://mavlink.io/en/messages/common.html#MAV_BATTERY_CHARGE_STATE
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_UNDEFINED:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_UNDEFINED'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_OK:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_OK'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_LOW:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_LOW'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_CRITICAL:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_CRITICAL'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_EMERGENCY:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_EMERGENCY'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_FAILED:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_FAILED'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_UNHEALTHY:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_UNHEALTHY'
            if message.charge_state == mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_CHARGING:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_CHARGING'
            else:
                self.battery_charge_state = 'MAV_BATTERY_CHARGE_STATE_UNDEFINED'
            return

        # MISSION_CURRENT tells us about our current mission waypoint
        if message.get_type() == 'MISSION_CURRENT':
            if not self.current_mission_item == message.seq:
                self.current_mission_item = message.seq
                self.current_mission_item_timestamp = time.time()
                print('Current mission item: ' + str(self.current_mission_item))
            return

        # MISSION_ITEM_REACHED tells us about the last reached mission item
        if message.get_type() == 'MISSION_ITEM_REACHED':
            if not self.last_reached_mission_item == message.seq:
                self.last_reached_mission_item = message.seq
                self._last_reached_mission_item_timestamp = time.time()
                print('Reached mission item: ' + str(self._last_reached_mission_item['number']))
            return

        # POSITION_TARGET_GLOBAL_INT tells us about our target position (when we have one)
        if message.get_type() == 'POSITION_TARGET_GLOBAL_INT':
            self.target_location_latitude = message.lat_int * 1e-7
            self.target_location_longitude = message.lon_int * 1e-7
            self.target_location_alt_abs = message.alt
            return

        # GPS_RAW_INT tells us about GPS stuff
        if message.get_type() == 'GPS_RAW_INT':
            self.GPS_status_satellites_visible = message.satellites_visible
            self.GPS_status_time_usec = message.time_usec
            self.GPS_status_fix_type = message.fix_type  # See https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
            if self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_NO_GPS:
                self.GPS_status_fix_type_description = 'NO_GPS'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_NO_FIX:
                self.GPS_status_fix_type_description = 'NO_FIX'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_2D_FIX:
                self.GPS_status_fix_type_description = '2D_FIX'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_3D_FIX:
                self.GPS_status_fix_type_description = '3D_FIX'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_DGPS:
                self.GPS_status_fix_type_description = 'DGPS'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_RTK_FLOAT:
                self.GPS_status_fix_type_description = 'RTK_FLOAT'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_RTK_FIXED:
                self.GPS_status_fix_type_description = 'RTK_FIXED'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_STATIC:
                self.GPS_status_fix_type_description = 'STATIC'
            elif self.GPS_status_fix_type == mavutil.mavlink.GPS_FIX_TYPE_PPP:
                self.GPS_status_fix_type_description = 'PPP'
            else:
                self.GPS_status_fix_type_description = 'Unknown'

    def process_status_text(self, message):
        if message.get_type() == 'STATUSTEXT':
            print('STATUSTEXT from ' + str(message.get_srcSystem()) + ':' + str(
                message.get_srcComponent()) + ': ' + message.text)


    def _arm_disarm(self, arm=True, force=False, timeout_sec=5, verbose=True, command_long_timeout=0.5):

        if arm:  # Request is to arm
            if force:
                params = [1, 21196, 0, 0, 0, 0, 0]
            else:
                params = [1, 0, 0, 0, 0, 0, 0]
        else:  # Request is to disarm
            if force:
                params = [0, 21196, 0, 0, 0, 0, 0]
            else:
                params = [0, 0, 0, 0, 0, 0, 0]

        # run_command_long can raise TimeoutException, catch it here
        try:
            response = self.run_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, params, timeout_sec=timeout_sec,
                                         verbose=verbose)
        except TimeoutException:
            return False

        if response['success'] == True:
            return True
        else:
            return False

    def arm(self, force=False, verbose=True):

        if self.armed:
            print('Vehicle already armed, not sending anything')
            return None

        result = self._arm_disarm(arm=True, force=force, verbose=verbose)

        return result

    def disarm(self, force=False, verbose=True):

        if not self.armed:
            print('Vehicle already disarmed, not sending anything')
            return None

        result = self._arm_disarm(arm=False, force=force, verbose=verbose)

        return result

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

        if self.autopilot_system_id is None:
            raise ValueError('autopilot_system_id not set yet')

        if self.autopilot_component_id is None:
            raise ValueError('autopilot_component_id not set yet')

        # First, let's construct our command
        cmd = self.autopilot_mavlink_handler.connection.mav.command_long_encode(self.autopilot_system_id,
                                                                              self.autopilot_component_id,
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
        ack = self.autopilot_mavlink_handler.send_get_response(cmd, 'COMMAND_ACK', timeout_sec=timeout_sec)
        dt = time.time() - t0

        if ack.result == expected_result:
            if verbose:
                s = ('%s: got successful ACK: %s in %.4f seconds' % (self.autopilot_mavlink_handler.name, mavutil.mavlink.enums["MAV_RESULT"][ack.result].name, dt))
                if self.autopilot_mavlink_handler.logger is None:
                    print(s)
                else:
                    self.autopilot_mavlink_handler.logger.info(s)
            return {'success': True, 'ack': ack}
        else:
            error_string = "%s: Expected %s got %s\nDescription:\n%s" % (
                self.autopilot_mavlink_handler.name,
                mavutil.mavlink.enums["MAV_RESULT"][expected_result].name,
                mavutil.mavlink.enums["MAV_RESULT"][ack.result].name,
                mavutil.mavlink.enums["MAV_RESULT"][ack.result].description
            )
            if verbose:
                if self.autopilot_mavlink_handler.logger is None:
                    print(error_string)
                else:
                    self.autopilot_mavlink_handler.logger.info(error_string)
            return {'success': False, 'ack': ack}


    def identify_autopilot(self, wait_loop_period_sec=0.01):
        '''
        Waits for a HEARTBEAT from a component identifying as mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        Sets fields autopilot_system_id, autopilot_component_id, autopilot_class accordingly.
        '''

        # Grab a HEARTBEAT from a component identifying as mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        # Set blocking=False to get get_next_message() to return a MessageRequest object
        heartbeat_request = ah.autopilot_mavlink_handler.history.get_next_message(
            message_type='HEARTBEAT', component_id=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, blocking=False)

        print('Blocking until we receive a heartbeat from an autopilot')
        while heartbeat_request.message is None:    # message will stop being None when an appropriate message arrives
            time.sleep(wait_loop_period_sec)

        self.autopilot_system_id = heartbeat_request.message.get_srcSystem()
        self.autopilot_component_id = heartbeat_request.message.get_srcComponent()
        self.autopilot_class = heartbeat_request.message.autopilot

        # We need to set this or we cannot send messages !!!
        self.autopilot_mavlink_handler.set_system_id(self.autopilot_system_id)

        # Reaching here means we have heartbeat from something identifying as an autopilot in heartbeat_request.message
        # print('Got HEARTBEAT from:\nSystem: ' + str(heartbeat_request.message.get_s))
        print('\nIdentified autopilot with:\nSystem ID: ' + str(self.autopilot_system_id) + '\nComponent ID: ' + str(
            self.autopilot_component_id) + '\nAutopilot class (MAV_AUTOPILOT): ' + str(self.autopilot_class) + '\n')


if __name__ == "__main__":

    # Connection related stuff
    connection_string = 'udpin:localhost:14550'
    component_id = 191  # companion computer but not enumerated in mavutil, see: https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ONBOARD_COMPUTER


    # NOTE: by NOT providing a system id below, we cannot send messages until our system id is set. This is done by calling
    # identify_autopilot() which sets the "system_id" field to that of the first received HEARTBEAT from an autopilot.
    # If we had wanted to send messages by setting our own system id, we should provide kwarg "system_id=..." below
    ah = AutopilotHandler(connection_string, component_id=component_id)

    # Once we have created our autopilot handler, we call identify_autopilot() to listen for a HEARTBEAT with
    # component_id of 1 (autopilot).
    # This will set ah.autopilot_system_id to the system id in this HEARTBEAT.
    # Subsequent messages sent out by the autopilot handler's mavlink handler will use this system id.
    ah.identify_autopilot()

    # Demonstrate arming the vehicle
    print('Trying to arm the vehicle')
    result = ah.arm()
    if result:
        print('Vehicle armed successfully')
    else:
        print('Vehicle failed to arm')
