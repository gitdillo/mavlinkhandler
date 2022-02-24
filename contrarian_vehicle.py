from pymavlink import mavutil
import time
from mavlinkhandler import MavlinkHandler
import threading

connection_string = 'udpout:localhost:14600'
my_system_id = 1
my_component_id = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

def refuse_arm(msg):
    if msg.get_type() == 'COMMAND_LONG' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        # Refuse arm / disarm commands by sending a COMMAND_ACK encoding failure
        ack_fail = mh.connection.mav.command_ack_encode(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                                        mavutil.mavlink.MAV_RESULT_FAILED)
        mh.send_message(ack_fail)
        # Also send a STATUS TEXT for good measure
        s = mh.connection.mav.statustext_encode(mavutil.mavlink.MAV_SEVERITY_INFO, b"Nope")
        mh.send_message(s)

def send_heartbeat(handler, period_sec):
    while True:
        h = handler.connection.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_KITE,
                                               mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        handler.send_message(h)
        time.sleep(period_sec)


# Start our mavlink handler
mh = MavlinkHandler()
mh.mavlink_update_thread.add_hook(refuse_arm)   # add our contrarian hook
mh.connect(connection_string=connection_string, source_system=my_system_id, source_component=my_component_id, start_update_thread=True)

# Start a thread sending a heartbeat
heartbeat_thread = threading.Thread(target=send_heartbeat, args=(mh, 1))
heartbeat_thread.daemon = True
heartbeat_thread.start()

