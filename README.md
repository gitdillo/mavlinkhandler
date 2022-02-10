# mavlinkhandler

First commit, if this does not destroy your computer and everything you have ever loved, consider yourself lucky.

Only known dependency is pymavlink.

Tested on kubuntu 20.04.3 LTS using python3

For a demo, **first** run `listener.py` in one terminal and **then**, in another terminal, run `initiator.py`

If you run in interactive mode (python -i ...), you can then use the `mh.send_message()` to send existing messages (there is a heartbeat named `h` lying about so `mh.send_message(h)` will work).

You can also make your own messages using the `mh.connection.mav.WHATEVER_encode(...)` and then send them across using `mh.send_message()`

Quick and dirty tip, in the python console type `mh.connection.mav.` and then double tap Tab to see the ...encode methods.
