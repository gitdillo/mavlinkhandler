from pymavlink import mavutil
from mavlinkhandler import MavlinkHandler

connection_string = 'udpin:localhost:14600'
my_system_id = 255
my_component_id = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER

def blip(m):
    if m.get_type() == 'HEARTBEAT':
        pass
        # Uncomment line below to get incoming heartbeats printed out
        # print('Heartbeat from ' + str(m.get_srcSystem()) + ':' + str(m.get_srcComponent()))
    elif m.get_type() == 'STATUSTEXT':
        print('Received STATUS TEXT: "' + m.text + '" from ' + str(m.get_srcSystem()) + ':' + str(m.get_srcComponent()))
    elif m.get_type() == 'COMMAND_ACK':
        print('Received COMMAND_ACK: ' + str(m) + '" from ' + str(m.get_srcSystem()) + ':' + str(m.get_srcComponent()))


if __name__ == "__main__":

    mh = MavlinkHandler()
    mh.mavlink_update_thread.add_hook(blip)     # add our hook to our mavlink update thread
    mh.connect(connection_string=connection_string, source_system=my_system_id, source_component=my_component_id, start_update_thread=True)

    # Let's twiddle our thumbs till we hear a heartbeat
    print('Patiently waiting for a heartbeat')
    while True:
        h_in = mh.history.wait_heartbeat(timeout_sec=5)
        if h_in is None:
            print("Don't mind me, I 'm fine, really, sitting here, all alone, waiting for a heartbeat...")
        else:
            # Save the incoming ID
            remote_id = (h_in.get_srcSystem(), h_in.get_srcComponent())     # system:component as a tuple
            # Let's encode a HEARTBEAT of our own
            h_out = mh.connection.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,
                                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            print('Responding with a heartbeat of our own')
            mh.send_message(h_out)
            break

    # Let's make a COMMAND_LONG: https://mavlink.io/en/messages/common.html#COMMAND_LONG
    # Ordering a MAV_CMD_COMPONENT_ARM_DISARM: https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    arm_command_long = mh.connection.mav.command_long_encode(remote_id[0],
                                                             remote_id[1],
                                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                                             0,
                                                             1,
                                                             0,
                                                             0,
                                                             0,
                                                             0,
                                                             0,
                                                             0)
    print('Sending ARM request to ' + str(remote_id[0]) + ':' + str(remote_id[1]))
    mh.send_message(arm_command_long)