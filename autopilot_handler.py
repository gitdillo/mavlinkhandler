from pymavlink import mavutil
import time
from mavlinkhandler import MavlinkHandler

class AutopilotHandler(object):
    '''
    A simple class monitoring a mavlink stream from an autopilot, updating some internal fields and offering an API for
    some commands.

    Typically, this would run in a companion computer but might also run in a GCS.
    '''
    def __init__(self, autopilot_connection_string, system_id=None, component_id=None):
        self.autopilot_mavlink_handler = MavlinkHandler()
        self.autopilot_mavlink_handler.connect(connection_string=autopilot_connection_string, start_update_thread=True,
                                               source_system=system_id, source_component=component_id)

        # Fields below are updated in update_fields
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
                self.logger.info(str(self.name) + ': state changed to: ' + str(self.mav_state))
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



# Connection related stuff
connection_string = 'udpin:localhost:14550'
component_id = 191  # companion computer but not enumerated in mavutil, see: https://mavlink.io/en/messages/common.html#MAV_COMP_ID_ONBOARD_COMPUTER


ah = AutopilotHandler(connection_string, component_id=component_id)

# Let's wait for a message from an autopilot and grab its system id.
# We will use it as our own system ID since we 'd like to believe we are companion computer onboard a vehicle
while True:
    print('Patiently waiting for any mavlink message from an autopilot')
    # We need messages from autopilots: mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 (value is 1)
    msg = ah.autopilot_mavlink_handler.history.get_next_message(component_id=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                                                              timeout_sec=10)
    if msg is not None:     # we got a message from a component identifying as an autopilot
        ah.autopilot_mavlink_handler.set_system_id(msg.get_srcSystem())     # set our system id to that of the message
        print(
            'Got: ' + str(msg.get_type()) + ', set system id to ' + str(ah.autopilot_mavlink_handler.get_system_id()))
        break