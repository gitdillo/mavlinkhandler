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
    if msg is not None:
        ah.autopilot_mavlink_handler.set_system_id(msg.get_srcSystem())
        print(
            'Got: ' + str(msg.get_type()) + ', set system id to ' + str(ah.autopilot_mavlink_handler.get_system_id()))
        break