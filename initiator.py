from pymavlink import mavutil
import time
from mavlinkhandler import MavlinkHandler

# We will use these parameters for our conection
# connection_string = 'udpin:localhost:14600'
connection_string = 'udpout:localhost:14600'
my_system_id = 1
my_component_id = 191   # companion computer: https://mavlink.io/en/messages/common.html#MAV_COMPONENT

# We will use these to draw attention to rare works of literature
dialogue = ['What is your name?',
             'What is your quest?',
             'What is your favorite color?',
             'Right. Off you go.'
             ]

def blip(m, mh):
    if m.get_type() == 'HEARTBEAT':
        print('Heartbeat from ' + str(m.get_srcSystem()) + ':' + str(m.get_srcComponent()))
    elif m.get_type() == 'STATUSTEXT':
        print('====> ' + m.text)


mh = MavlinkHandler()
mh.mavlink_update_thread.add_hook(blip)
mh.connect(connection_string=connection_string, source_system=my_system_id, source_component=my_component_id, start_update_thread=True)


# Let's encode a HEARTBEAT
# Our MavlinkHandler object gives us access to pymavlink's xxx_encode() methods via its .connection.mav field
h = mh.connection.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)

# Send the heartbeat to the other party (assumes the other party up and running)
mh.send_message(h)
# we wait silently for a heartbeat, not like that whiner, the listener
while True:
    h = mh.history.wait_heartbeat()     # whatever the default timeout is
    if h:
        break

# OK, reaching here means we have exchanged pleasantries so the other side has our attention, let's hit them
for d in dialogue:
    s=mh.connection.mav.statustext_encode(mavutil.mavlink.MAV_SEVERITY_INFO, bytes(d, 'ascii'))
    print('<==== ' + s.text.decode('ascii'))    # Print what we are sending
    mh.send_message(s)
    s = mh.history.get_next_message('STATUSTEXT', timeout_sec=5)    # wait for a response
    if s is None:   # no response by end of timeout, should not happen if listener is working properly
        print('The keeper has died, I am crossing the bridge')
    time.sleep(1)   # wait a bit before answering
