from omni import OmniComms
import numpy as np
import time
import threading


class JohnRobot(object):
    """Class that controls John's old robot he made in high school. Essentially
    the same as robocup except larger, slightly different wheels, and different
    arduino used to control. Commands should still be in the same format"""

    def __init__(self):
        self.comms = OmniComms()

    def move(self, forward, lateral, w, time=0.5):
        """Move forward, laterally, and rotation. Should be given as a int
        from 0 to 255, with 255 being the fastest."""
        # Use -1 as first element to broadcast to all robots
        robot_id = -1
        f = int(np.clip(forward, -255, 255))
        l = int(np.clip(lateral, -255, 255))
        w = int(np.clip(w, -255, 255))
        time_ms = int(time * 1000.0)
        cmd = "{},{},{},{},{}".format(robot_id, l, f, w, time_ms)
        self.comms.send(cmd)

    def die(self):
        self.comms.close()


if __name__ == '__main__':
    test = JohnRobot()
    for _ in range(3):
        test.move(30, 0, 0, time=1.0)
        time.sleep(0.9)
        test.move(0, 30, 0, time=1.0)
        time.sleep(0.9)
        test.move(-30, 0, 0, time=1.0)
        time.sleep(0.9)
        test.move(0, -30, 0, time=1.0)
        time.sleep(0.9)
    test.die()
