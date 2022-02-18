import sys
import time
import numpy as np

from pewpew.planner import MotionPlanner, KinematicLimits
from pewpew import MachineConnection
from pewpew.definitions import MessageType
if len(sys.argv) < 2:
    print("Usage: example.py <path to usb device>")
    exit()

with MachineConnection(sys.argv[-1]) as m:

    # Wait until the machine sends us a valid status
    status = None
    while not status:
        status = m.status()
        time.sleep(0.25)

    # Build a motion planner with the right number of axes and some fake microsteps/velocities limits/acceleration limits
    ones = np.ones(len(status.position))
    limits = KinematicLimits(v_max = 5 * ones, a_max = 10 * ones, junction_speed = 0.05, junction_deviation = 0.1)
    planner = MotionPlanner(limits, microsteps = 100 * ones, position = np.zeros_like(ones))
    planner.set_position(status.position, microsteps = True)
    
    # Trigger a move to (1,1,...) and then back to (0,0....)
    m.buffered_messages(planner.goto(*ones))
    m.buffered_messages(planner.goto(*np.zeros_like(ones)))
    m.wait_until_idle()


    for i in range(10):
        m.realtime_message(MessageType.QUIZ)
        print(m.signals.peripheral)
        time.sleep(0.5)
