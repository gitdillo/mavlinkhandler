from pymavlink import mavutil
import time
from mavlinkhandler import MavlinkHandler


# We will use these parameters for our conection
connection_string = 'udpin:localhost:14600'
# connection_string = 'udpout:localhost:14600'
my_system_id = 2
my_component_id = 191   # companion computer: https://mavlink.io/en/messages/common.html#MAV_COMPONENT

# We will use these to draw attention to rare works of literature
dialogue = ['My name is Sir Lancelot of Camelot.',
             'To seek the Holy Grail.',
             'Blue.',
             'Oh, thank you. Thank you very much.'
             ]



def blip(m, mh):
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

# Reaching here means the other side will hit us with their side of the dialogue so let's listen for it
dialogue_counter = 0
while True:
    s = mh.history.get_next_message('STATUSTEXT', timeout_sec=5)
    if s is None:
        print('The other side seems to have dropped out of the conversation')
        break
    time.sleep(1)   # wait a bit before answering
    s = mh.connection.mav.statustext_encode(mavutil.mavlink.MAV_SEVERITY_INFO,
                                            bytes(dialogue[dialogue_counter], 'ascii'))
    print('<==== ' + s.text.decode('ascii'))    # Print what we are sending
    mh.send_message(s)
    dialogue_counter += 1