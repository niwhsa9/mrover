#!/usr/bin/env python3
import threading
import time as t
import asyncio
import rospy
import math
from geometry_msgs.msg import Twist
from typing import NoReturn
from sensor_msgs.msg import JointState
from mrover.msg import DriveStatus, MoteusState as MoteusStateMsg
import moteus
import moteus.multiplex as mp
import io


class CommandData:
    DEFAULT_TORQUE = 0.3
    MAX_TORQUE = 0.5
    POSITION_FOR_VELOCITY_CONTROL = math.nan
    VELOCITY_LIMIT_REV_S = 50  # DO NOT CHANGE THIS HAPHAZARDLY. DERIVED FROM TESTING.
    ZERO_VELOCITY = 0.0

    def __init__(
        self,
        position: float = POSITION_FOR_VELOCITY_CONTROL,
        velocity: float = ZERO_VELOCITY,
        torque: float = DEFAULT_TORQUE,
    ):
        self.position = position
        self.velocity = max(-CommandData.VELOCITY_LIMIT_REV_S, min(CommandData.VELOCITY_LIMIT_REV_S, velocity))
        self.torque = max(0, min(CommandData.MAX_TORQUE, torque))


class MoteusData:
    def __init__(self, position: float, velocity: float, torque: float):
        self.position = position
        self.velocity = velocity
        self.torque = torque


class MoteusState:
    DISCONNECTED_STATE = "Disconnected"
    ARMED_STATE = "Armed"
    ERROR_STATE = "Error"
    MOTEUS_UNPOWERED_OR_NOT_FOUND_ERROR = 99  # a custom error for when the moteus is unpowered or not found

    ERROR_CODE_DICTIONARY = {
        1: "DmaStreamTransferError",
        2: "DmaStreamFifiError",
        3: "UartOverrunError",
        4: "UartFramingError",
        5: "UartNoiseError",
        6: "UartBufferOverrunError",
        7: "UartParityError",
        32: "CalibrationFault",
        33: "MotorDriverFault",
        34: "OverVoltage",
        35: "EncoderFault",
        36: "MotorNotConfigured",
        37: "PwmCycleOverrun",
        38: "OverTemperature",
        39: "StartOutsideLimit",
        40: "UnderVoltage",
        41: "ConfigChanged",
        42: "ThetaInvalid",
        43: "PositionInvalid",
        MOTEUS_UNPOWERED_OR_NOT_FOUND_ERROR: "Moteus Unpowered or Not Found",
    }

    NO_ERROR_NAME = "No Error"

    def __init__(self, state: str, error_name: str):
        self.state = state
        self.error_name = error_name


def is_fault_response_an_error(fault_response: int) -> bool:
    """
    Returns True if the fault response is an error
    :param fault_response: the value read in from the fault register of the moteus CAN message
    :return: True if the fault response is an error
    """
    return fault_response != 0


class MoteusBridge:

    MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S = 0.01
    ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.1

    def __init__(self, can_id: int, transport):

        self._can_id = can_id
        self.controller = moteus.Controller(id=can_id, transport=transport)
        self.command_lock = threading.Lock()
        self._command = CommandData(
            position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
            velocity=CommandData.ZERO_VELOCITY,
            torque=CommandData.MAX_TORQUE,
        )
        self.moteus_state = MoteusState(state=MoteusState.DISCONNECTED_STATE, error_name=MoteusState.NO_ERROR_NAME)
        self.moteus_data = MoteusData(position=math.nan, velocity=math.nan, torque=math.nan)

    def set_command(self, command: CommandData) -> None:
        """
        Changes the values of the arguments of set_position for when it is next called.
        :param command: contains arguments that are used in the moteus controller.set_position function
        :return: none
        """
        with self.command_lock:
            self._command = command

    async def _send_desired_command(self) -> None:
        """
        Calls controller.set_position (which controls the moteus over CAN) with previously the most recent requested
        commands. If the message has not been received within a certain amount of time, disconnected state is entered.
        Otherwise, error state is entered. State must be in armed state.
        """
        assert self.moteus_state.state == MoteusState.ARMED_STATE

        with self.command_lock:
            command = self._command

        try:
            await self._send_command(command)
        except asyncio.TimeoutError:
            if self.moteus_state.state != MoteusState.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to send command")
            self._change_state(MoteusState.DISCONNECTED_STATE)

    async def _send_command(self, command: CommandData) -> None:
        """
        Calls controller.set_position (which controls the moteus over CAN) with the requested command.
        If the message has not been received within a certain amount of time, disconnected state is entered.
        Otherwise, error state is entered.

        It is expected that this may throw an error if there is a timeout.
        """
        # Check if our commanded velocity is close to zero
        # We can't compare floats directly due to representation so use tolerance
        if abs(command.velocity) > 1e-5:
            state = await asyncio.wait_for(
                self.controller.set_position(
                    position=command.position,
                    velocity=command.velocity,
                    velocity_limit=CommandData.VELOCITY_LIMIT_REV_S,
                    maximum_torque=command.torque,
                    watchdog_timeout=MoteusBridge.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
                    query=True,
                ),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
        else:
            # await self.controller.set_stop()
            await asyncio.wait_for(
                MoteusBridge.set_brake(self.controller),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )
            state = await asyncio.wait_for(
                self.controller.query(),
                timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S,
            )

        moteus_not_found = state is None or not hasattr(state, "values") or moteus.Register.FAULT not in state.values
        if moteus_not_found:
            self._handle_error(MoteusState.MOTEUS_UNPOWERED_OR_NOT_FOUND_ERROR)
        else:

            is_error = is_fault_response_an_error(state.values[moteus.Register.FAULT])

            if is_error:
                self._handle_error(state.values[moteus.Register.FAULT])
            else:
                self._change_state(MoteusState.ARMED_STATE)

            self.moteus_data = MoteusData(
                position=state.values[moteus.Register.POSITION],
                velocity=state.values[moteus.Register.VELOCITY],
                torque=state.values[moteus.Register.TORQUE],
            )

    def _handle_error(self, fault_response: int) -> None:
        """
        Handles error by changing the state to error state
        :param fault_response: the value read in from the fault register of the moteus CAN message.
        Or a custom error, 99.
        """
        assert is_fault_response_an_error(fault_response)

        self._change_state(MoteusState.ERROR_STATE)

        try:
            error_description = MoteusState.ERROR_CODE_DICTIONARY[fault_response]
        except KeyError:
            error_description = MoteusState.NO_ERROR_NAME

        if self.moteus_state.error_name != error_description:
            rospy.logerr(f"CAN ID {self._can_id} has encountered an error: {error_description}")
            self.moteus_state.error_name = error_description

    async def _connect(self) -> None:
        """
        Attempts to establish a connection to the moteus. State must be in error or disconnected state.
        """
        try:
            assert self.moteus_state.state != MoteusState.ARMED_STATE

            await asyncio.wait_for(
                self.controller.set_stop(), timeout=self.MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S
            )

            command = CommandData(
                position=math.nan,
                velocity=CommandData.ZERO_VELOCITY,
                torque=CommandData.DEFAULT_TORQUE,
            )
            await self._send_command(command)

        except asyncio.TimeoutError:
            if self.moteus_state.state != MoteusState.DISCONNECTED_STATE:
                rospy.logerr(f"CAN ID {self._can_id} disconnected when trying to connect")
            self._change_state(MoteusState.DISCONNECTED_STATE)

    def _change_state(self, state: str) -> None:
        """
        Changes the state as needed. If no longer armed, then the data will be set to math.nan.
        If no longer in an error state, error name is set to no error.
        :param state: Must be disconnected, armed, or error
        """

        if state == MoteusState.ARMED_STATE:
            self.moteus_state.state = MoteusState.ARMED_STATE
            self.moteus_state.error_name = MoteusState.NO_ERROR_NAME
        elif state == MoteusState.DISCONNECTED_STATE:
            self.moteus_state.state = MoteusState.DISCONNECTED_STATE
            self.moteus_state.error_name = MoteusState.NO_ERROR_NAME
            self.moteus_data = MoteusData(position=math.nan, velocity=math.nan, torque=math.nan)
        else:
            self.moteus_state.state = MoteusState.ERROR_STATE
            self.moteus_data = MoteusData(position=math.nan, velocity=math.nan, torque=math.nan)

    async def update(self) -> None:
        """
        Determines actions based on state. If in armed state, commands will be sent to the moteus.
        If in disconnected or error state, attempts will be made to return to armed state.
        """
        if self.moteus_state.state == MoteusState.ARMED_STATE:
            await self._send_desired_command()
        elif (
            self.moteus_state.state == MoteusState.DISCONNECTED_STATE
            or self.moteus_state.state == MoteusState.ERROR_STATE
        ):
            await self._connect()

    @staticmethod
    def make_brake(controller, *, query=False):
        """
        Temporary fix, taken from https://github.com/mjbots/moteus/blob/335d40ef2b78335be89f27fbb27c94d1a1333b25/lib/python/moteus/moteus.py#L1027
        The problem is the Python 3.7 moteus library does not have set_brake and that is the version the Pi has.
        """
        STOPPED_MODE: int = 15

        result = controller._make_command(query=query)

        data_buf = io.BytesIO()
        writer = mp.WriteFrame(data_buf)
        writer.write_int8(mp.WRITE_INT8 | 0x01)
        writer.write_int8(int(moteus.Register.MODE))
        writer.write_int8(STOPPED_MODE)

        if query:
            data_buf.write(controller._query_data)

        result.data = data_buf.getvalue()

        return result

    @staticmethod
    async def set_brake(controller, *args, **kwargs):
        return await controller.execute(MoteusBridge.make_brake(controller, **kwargs))


class DriveApp:
    BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S = 1
    DRIVE_NAMES = ["FrontLeft", "FrontRight", "MiddleLeft", "MiddleRight", "BackLeft", "BackRight"]

    def __init__(self):
        self.drive_info_by_name = rospy.get_param("brushless/drive")
        self.drive_bridge_by_name = {}
        using_pi3_hat = rospy.get_param("brushless/using_pi3_hat")
        if using_pi3_hat:
            import moteus_pi3hat

            transport = moteus_pi3hat.Pi3HatRouter(
                servo_bus_map={1: [info["id"] for name, info in self.drive_info_by_name.items()]}
            )
        else:
            transport = None

        for name, info in self.drive_info_by_name.items():
            self.drive_bridge_by_name[name] = MoteusBridge(info["id"], transport)
        rover_width = rospy.get_param("rover/width")
        rover_length = rospy.get_param("rover/length")
        self.WHEEL_DISTANCE_INNER = rover_width / 2.0
        self.WHEEL_DISTANCE_OUTER = math.sqrt(((rover_width / 2.0) ** 2) + ((rover_length / 2.0) ** 2))

        ratio_motor_to_wheel = rospy.get_param("wheel/gear_ratio")

        # To convert m/s to rev/s, multiply by this constant. Divide by circumference, multiply by gear ratio.
        self.WHEELS_M_S_TO_MOTOR_REV_S = (1 / (rospy.get_param("wheel/radius") * 2 * math.pi)) * ratio_motor_to_wheel

        _max_speed_m_s = rospy.get_param("rover/max_speed")
        assert _max_speed_m_s > 0, "rover/max_speed config must be greater than 0"

        self._max_motor_speed_rev_s = _max_speed_m_s * self.WHEELS_M_S_TO_MOTOR_REV_S
        self._last_updated_time = t.time()

        rospy.Subscriber("cmd_vel", Twist, self._process_twist_message)

        self._drive_status_publisher = rospy.Publisher("drive_status", DriveStatus, queue_size=1)
        self._drive_status = DriveStatus(
            name=[name for name in DriveApp.DRIVE_NAMES],
            joint_states=JointState(
                name=[name for name in DriveApp.DRIVE_NAMES],
                position=[self.drive_bridge_by_name[name].moteus_data.position for name in DriveApp.DRIVE_NAMES],
                velocity=[self.drive_bridge_by_name[name].moteus_data.velocity for name in DriveApp.DRIVE_NAMES],
                effort=[self.drive_bridge_by_name[name].moteus_data.torque for name in DriveApp.DRIVE_NAMES],
            ),
            moteus_states=MoteusStateMsg(
                name=[name for name in DriveApp.DRIVE_NAMES],
                state=[self.drive_bridge_by_name[name].moteus_state.state for name in DriveApp.DRIVE_NAMES],
                error=[self.drive_bridge_by_name[name].moteus_state.error_name for name in DriveApp.DRIVE_NAMES],
            ),
        )

    def _process_twist_message(self, ros_msg: Twist) -> None:
        """
        Converts a Twist message to individual motor speeds.
        :param ros_msg: Has linear x and angular z velocity components.
        Linear velocity is assumed to be in meters per second.
        Angular velocity is assumed to be in radians per second.
        """
        forward = ros_msg.linear.x
        turn = ros_msg.angular.z

        # Multiply radians per second by the radius and you get arc length because s = r(theta).
        turn_difference_inner = turn * self.WHEEL_DISTANCE_INNER
        turn_difference_outer = turn * self.WHEEL_DISTANCE_OUTER

        left_rev_inner = (forward - turn_difference_inner) * self.WHEELS_M_S_TO_MOTOR_REV_S
        right_rev_inner = (forward + turn_difference_inner) * self.WHEELS_M_S_TO_MOTOR_REV_S
        left_rev_outer = (forward - turn_difference_outer) * self.WHEELS_M_S_TO_MOTOR_REV_S
        right_rev_outer = (forward + turn_difference_outer) * self.WHEELS_M_S_TO_MOTOR_REV_S

        # Ignore inner since outer > inner always
        larger_abs_rev_s = max(abs(left_rev_outer), abs(right_rev_outer))
        if larger_abs_rev_s > self._max_motor_speed_rev_s:
            change_ratio = self._max_motor_speed_rev_s / larger_abs_rev_s
            left_rev_inner *= change_ratio
            right_rev_inner *= change_ratio
            left_rev_outer *= change_ratio
            right_rev_outer *= change_ratio

        self._last_updated_time = t.time()
        for name, bridge in self.drive_bridge_by_name.items():

            if name == "FrontLeft":
                front_left_multiplier = self.drive_info_by_name[name]["multiplier"]
                commanded_velocity_rev_s = front_left_multiplier * left_rev_outer
            elif name == "BackLeft":
                back_left_multiplier = self.drive_info_by_name[name]["multiplier"]
                commanded_velocity_rev_s = back_left_multiplier * left_rev_outer
            elif name == "MiddleLeft":
                middle_left_multiplier = self.drive_info_by_name[name]["multiplier"]
                commanded_velocity_rev_s = middle_left_multiplier * left_rev_inner
            elif name == "FrontRight":
                front_right_multiplier = self.drive_info_by_name[name]["multiplier"]
                commanded_velocity_rev_s = front_right_multiplier * right_rev_outer
            elif name == "BackRight":
                back_right_multiplier = self.drive_info_by_name[name]["multiplier"]
                commanded_velocity_rev_s = back_right_multiplier * right_rev_outer
            elif name == "MiddleRight":
                middle_right_multiplier = self.drive_info_by_name[name]["multiplier"]
                commanded_velocity_rev_s = middle_right_multiplier * right_rev_inner
            else:
                rospy.logerr(f"Invalid name {name}")
                continue

            if bridge.moteus_state.state == MoteusState.ARMED_STATE:
                bridge.set_command(
                    CommandData(
                        position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
                        velocity=commanded_velocity_rev_s,
                        torque=CommandData.DEFAULT_TORQUE,
                    )
                )

    async def run(self) -> NoReturn:
        """
        Runs an infinite loop and only gives commands to the moteus (by updating the bridge) if communication is still
        maintained between the basestation and the rover node.
        """
        previously_lost_communication = True
        while not rospy.is_shutdown():
            for name, bridge in self.drive_bridge_by_name.items():
                time_diff_since_updated = t.time() - self._last_updated_time
                lost_communication = time_diff_since_updated > DriveApp.BASESTATION_TO_ROVER_NODE_WATCHDOG_TIMEOUT_S
                if lost_communication:
                    if not previously_lost_communication:
                        rospy.loginfo("Lost communication")
                        previously_lost_communication = True
                    bridge.set_command(
                        CommandData(
                            position=CommandData.POSITION_FOR_VELOCITY_CONTROL,
                            velocity=CommandData.ZERO_VELOCITY,
                            torque=CommandData.DEFAULT_TORQUE,
                        )
                    )
                elif previously_lost_communication:
                    previously_lost_communication = False
                    rospy.loginfo("Regained communication")

                await bridge.update()

                index = DriveApp.DRIVE_NAMES.index(name)
                self._drive_status.joint_states.position[index] = bridge.moteus_data.position
                self._drive_status.joint_states.velocity[index] = bridge.moteus_data.velocity
                self._drive_status.joint_states.effort[index] = bridge.moteus_data.torque
                self._drive_status.moteus_states.state[index] = bridge.moteus_state.state
                self._drive_status.moteus_states.error[index] = bridge.moteus_state.error_name

            self._drive_status_publisher.publish(self._drive_status)
        assert False


class Application:
    def __init__(self):
        rospy.init_node(f"brushless")
        self.drive_app = DriveApp()

    def run(self) -> NoReturn:
        """
        Allows the functions to run.
        :return:
        """
        asyncio.run(self.drive_app.run())


def main():
    app = Application()
    app.run()


if __name__ == "__main__":
    main()
