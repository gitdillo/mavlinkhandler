from pymavlink import mavutil
import time
from mavlinkhandler import MavlinkHandler

connection_string = 'udpin:localhost:14600'
my_system_id = 255
my_component_id = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER

def blip(m):
    if m.get_type() == 'HEARTBEAT':
        print('Heartbeat from ' + str(m.get_srcSystem()) + ':' + str(m.get_srcComponent()))
    elif m.get_type() == 'STATUSTEXT':
        print('====> ' + m.text)


mh = MavlinkHandler()
mh.mavlink_update_thread.add_hook(blip)     # add our hook to our mavlink update thread
mh.connect(connection_string=connection_string, source_system=my_system_id, source_component=my_component_id, start_update_thread=True)

# Let's twiddle our thumbs till we hear a heartbeat
print('Patiently waiting for a heartbeat')
while True:
    h = mh.history.wait_heartbeat(timeout_sec=5)
    if h is None:
        print("Don't mind me, I 'm fine, really, sitting here, all alone, waiting for a heartbeat...")
    else:
        # Let's encode a HEARTBEAT
        h = mh.connection.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                               mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        print('Responding with a heartbeat of our own')
        mh.send_message(h)
        break

# TODO: send request arm