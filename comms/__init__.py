try:
    from comms import Comms
    from robot_commands import RobotCommands
    from robot_status import RobotStatus
except (SystemError, ImportError):
    from .comms import Comms  # noqa
    from .robot_commands import RobotCommands   # noqa
    from .robot_status import RobotStatus  # noqa
