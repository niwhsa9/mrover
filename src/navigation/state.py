from geometry_msgs.msg import Twist

from util.state_lib.state import State

from navigation import waypoint


class DoneState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context):
        # Check if we have a course to traverse
        if context.course and (not context.course.is_complete()):
            return waypoint.WaypointState()

        # Stop rover
        cmd_vel = Twist()
        context.rover.send_drive_command(cmd_vel)
        return self


class OffState(State):
    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context):
        if context.course and (not context.course.is_complete()):
            return waypoint.WaypointState()

        cmd_vel = Twist()
        context.rover.send_drive_command(cmd_vel)
        return self


def off_check(context) -> bool:
    """
    function that state machine will call to check if the rover is turned off.
    """
    if context.disable_requested:
        context.disable_requested = False
        context.course = None
        context.rover.stuck = False
        context.rover.driver.reset()
        return True
    return False
