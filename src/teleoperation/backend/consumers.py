import csv
import json
import os
import threading
from datetime import datetime
from math import copysign, pi, isfinite
import cv2

import pytz
from channels.generic.websocket import JsonWebsocketConsumer

import rospy
import tf2_ros
from backend.models import AutonWaypoint, BasicWaypoint
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, PoseStamped

import actionlib

import actionlib

from cv_bridge import CvBridge
from mrover.msg import (
    ArmActionAction,
    ArmActionGoal,
    PDLB,
    ControllerState,
    GPSWaypoint,
    WaypointType,
    LED,
    StateMachineStateUpdate,
    Throttle,
    CalibrationStatus,
    MotorsStatus,
    CameraCmd,
    Velocity,
    Position,
    IK,
    Spectral,
    ScienceThermistors,
    HeaterData,
    CapturePanoramaAction,
    CapturePanoramaFeedback,
    CapturePanoramaGoal,
)
from mrover.srv import EnableAuton, AdjustMotor, ChangeCameras
from sensor_msgs.msg import NavSatFix, Temperature, RelativeHumidity, JointState
from std_msgs.msg import String, Header
from std_srvs.srv import SetBool, Trigger
from tf.transformations import euler_from_quaternion
from util.SE3 import SE3

import numpy as np

DEFAULT_ARM_DEADZONE = 0.15


def deadzone(signal: float, threshold: float) -> float:
    """
    Values lower than the threshold will be clipped to zero.
    Those above will be remapped to [0, 1] linearly.
    """
    magnitude = abs(signal)
    magnitude = 0 if magnitude < threshold else (magnitude - threshold) / (1 - threshold)
    return copysign(magnitude, signal)


def quadratic(signal: float) -> float:
    """
    Use to allow more control near low inputs values by squaring the magnitude.
    For example using a joystick to control drive.
    """
    return copysign(signal**2, signal)


tf2_buffer = tf2_ros.Buffer()
tf2_ros.TransformListener(tf2_buffer)


class GUIConsumer(JsonWebsocketConsumer):
    def connect(self):
        self.accept()
        try:
            # ROS Parameters
            self.joystick_mappings = rospy.get_param("teleop/joystick_mappings")
            self.drive_config = rospy.get_param("teleop/drive_controls")
            self.max_wheel_speed = rospy.get_param("rover/max_speed")
            self.wheel_radius = rospy.get_param("wheel/radius")
            self.max_angular_speed = self.max_wheel_speed / self.wheel_radius
            self.ra_config = rospy.get_param("teleop/ra_controls")
            self.ik_names = rospy.get_param("teleop/ik_multipliers")
            self.RA_NAMES = rospy.get_param("teleop/ra_names")
            self.brushless_motors = rospy.get_param("brushless_motors/controllers")
            self.brushed_motors = rospy.get_param("brushed_motors/controllers")
            self.xbox_mappings = rospy.get_param("teleop/xbox_mappings")
            self.sa_config = rospy.get_param("teleop/sa_controls")

            # Publishers
            self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
            self.led_pub = rospy.Publisher("/auton_led_cmd", String, queue_size=1)
            self.mast_gimbal_pub = rospy.Publisher("/mast_gimbal_throttle_cmd", Throttle, queue_size=1)
            self.arm_ik_pub = rospy.Publisher("arm_ik", IK, queue_size=1)
            self.arm_throttle_cmd_pub = rospy.Publisher("arm_throttle_cmd", Throttle, queue_size=1)
            self.arm_velocity_cmd_pub = rospy.Publisher("arm_velocity_cmd", Velocity, queue_size=1)
            self.arm_position_cmd_pub = rospy.Publisher("arm_position_cmd", Position, queue_size=1)
            self.sa_throttle_cmd_pub = rospy.Publisher("sa_throttle_cmd", Throttle, queue_size=1)
            self.sa_velocity_cmd_pub = rospy.Publisher("sa_velocity_cmd", Velocity, queue_size=1)
            self.sa_position_cmd_pub = rospy.Publisher("sa_position_cmd", Position, queue_size=1)
            self.cache_throttle_cmd_pub = rospy.Publisher("cache_throttle_cmd", Throttle, queue_size=1)
            self.cache_velocity_cmd_pub = rospy.Publisher("cache_velocity_cmd", Velocity, queue_size=1)
            self.cache_position_cmd_pub = rospy.Publisher("cache_position_cmd", Position, queue_size=1)

            # Subscribers
            self.pdb_sub = rospy.Subscriber("/pdb_data", PDLB, self.pdb_callback)
            self.arm_moteus_sub = rospy.Subscriber(
                "/arm_controller_data", ControllerState, self.arm_controller_callback
            )
            self.arm_joint_sub = rospy.Subscriber("/arm_joint_data", JointState, self.arm_joint_callback)
            self.sa_joint_sub = rospy.Subscriber("/sa_joint_data", JointState, self.sa_joint_callback)
            self.drive_moteus_sub = rospy.Subscriber(
                "/drive_controller_data", ControllerState, self.drive_controller_callback
            )
            self.gps_fix = rospy.Subscriber("/left_gps_driver/fix", NavSatFix, self.gps_fix_callback)
            self.drive_status_sub = rospy.Subscriber("/drive_status", MotorsStatus, self.drive_status_callback)
            self.led_sub = rospy.Subscriber("/led", LED, self.led_callback)
            self.nav_state_sub = rospy.Subscriber("/nav_state", StateMachineStateUpdate, self.nav_state_callback)
            self.imu_calibration = rospy.Subscriber("imu/calibration", CalibrationStatus, self.imu_calibration_callback)
            self.sa_temp_data = rospy.Subscriber("/sa_temp_data", Temperature, self.sa_temp_data_callback)
            self.sa_humidity_data = rospy.Subscriber(
                "/sa_humidity_data", RelativeHumidity, self.sa_humidity_data_callback
            )
            self.ish_thermistor_data = rospy.Subscriber(
                "/science_thermistors", ScienceThermistors, self.ish_thermistor_data_callback
            )
            self.ish_heater_state = rospy.Subscriber(
                "/science_heater_state", HeaterData, self.ish_heater_state_callback
            )
            self.science_spectral = rospy.Subscriber("/science_spectral", Spectral, self.science_spectral_callback)
            self.cmd_vel = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

            # Action clients
            self.pano_client = actionlib.SimpleActionClient("panorama", CapturePanoramaAction)

            # Services
            self.laser_service = rospy.ServiceProxy("enable_arm_laser", SetBool)
            self.enable_auton = rospy.ServiceProxy("enable_auton", EnableAuton)
            self.change_heater_srv = rospy.ServiceProxy("science_change_heater_auto_shutoff_state", SetBool)
            self.calibrate_cache_srv = rospy.ServiceProxy("cache_calibrate", Trigger)
            self.cache_enable_limit = rospy.ServiceProxy("cache_enable_limit_switches", SetBool)
            self.calibrate_service = rospy.ServiceProxy("arm_calibrate", Trigger)
            self.change_cameras_srv = rospy.ServiceProxy("change_cameras", ChangeCameras)
            self.heater_auto_shutoff_srv = rospy.ServiceProxy("science_change_heater_auto_shutoff_state", SetBool)

            self.flight_thread = threading.Thread(target=self.flight_attitude_listener)
            self.flight_thread.start()

            self.joystick_twist = np.zeros(2)
            self.xbox_twist = np.zeros(2)
        except Exception as e:
            rospy.logerr(e)

    def disconnect(self, close_code):
        self.pdb_sub.unregister()
        self.arm_moteus_sub.unregister()
        self.drive_moteus_sub.unregister()
        self.drive_status_sub.unregister()
        self.gps_fix.unregister()
        self.led_sub.unregister()
        self.nav_state_sub.unregister()
        self.imu_calibration.unregister()
        self.sa_temp_data.unregister()
        self.sa_humidity_data.unregister()
        self.ish_thermistor_data.unregister()
        self.science_spectral.unregister()

    def send_twist(self):
        combined = self.joystick_twist + self.xbox_twist
        self.twist_pub.publish(
            Twist(
                linear=Vector3(x=combined[0]),
                angular=Vector3(z=combined[1]),
            )
        )

    def receive(self, text_data=None, bytes_data=None, **kwargs):
        if text_data is None:
            rospy.logwarn("Expecting text but received binary on GUI websocket...")
            return

        message = json.loads(text_data)
        try:
            if message["type"] == "joystick_values":
                self.handle_joystick_message(message)
            elif message["type"] == "disable_auton_led":
                self.disable_auton_led()
            elif message["type"] == "laser_service":
                self.enable_laser_callback(message)
            elif message["type"] == "calibrate_motors":
                self.calibrate_motors(message)
            elif message["type"] == "arm_adjust":
                self.arm_adjust(message)
            elif message["type"] in {"arm_values", "cache_values", "sa_arm_values"}:
                self.handle_controls_message(message)
            elif message["type"] == "enable_white_leds":
                self.enable_white_leds_callback(message)
            elif message["type"] == "enable_uv_leds":
                self.enable_uv_leds_callback(message)
            elif message["type"] == "auton_command":
                self.send_auton_command(message)
            elif message["type"] == "teleop_enabled":
                self.send_teleop_enabled(message)
            elif message["type"] == "bearing":
                self.bearing()
            elif message["type"] == "mast_gimbal":
                self.mast_gimbal(message)
            elif message["type"] == "max_streams":
                self.send_res_streams()
            elif message["type"] == "sendCameras":
                self.change_cameras(message)
            elif message["type"] == "takePanorama":
                self.capture_panorama()
            elif message["type"] == "capturePhoto":
                self.capture_photo()
            elif message["type"] == "heaterEnable":
                self.heater_enable_service(message)
            elif message["type"] == "autoShutoff":
                self.auto_shutoff_toggle(message)
            elif message["type"] == "center_map":
                self.send_center()
            elif message["type"] == "enable_limit_switch":
                self.limit_switch(message)
            elif message["type"] == "save_auton_waypoint_list":
                self.save_auton_waypoint_list(message)
            elif message["type"] == "get_auton_waypoint_list":
                self.get_auton_waypoint_list(message)
            elif message["type"] == "save_basic_waypoint_list":
                self.save_basic_waypoint_list(message)
            elif message["type"] == "get_basic_waypoint_list":
                self.get_basic_waypoint_list(message)
            elif message["type"] == "download_csv":
                self.download_csv(message)
            elif message["type"] == "reset_gimbal":
                self.reset_gimbal()
        except Exception as e:
            rospy.logerr(e)

    @staticmethod
    def remap(value: float, old_min: float, old_max: float, new_min: float, new_max: float) -> float:
        return (value - old_min) * (new_max - new_min) / (old_max - old_min) + new_min

    @staticmethod
    def buttons_as_axis(buttons: list[float], mappings: dict[str, int], pos_button: str, neg_button: str) -> float:
        return buttons[mappings[pos_button]] - buttons[mappings[neg_button]]

    @staticmethod
    def filter_axis(
        axes: list[float],
        mappings: dict[str, int],
        name: str,
        deadzone_threshold: float = 0.05,
        apply_quadratic: bool = False,
        scale: float = 1.0,
    ) -> float:
        signal = axes[mappings[name]]
        signal = deadzone(signal, deadzone_threshold)
        if apply_quadratic:
            signal = quadratic(signal)
        signal *= scale
        return signal

    def to_velocity(self, input: int, joint_name: str, brushless: bool = True) -> float:
        """
        Scales [-1,1] joystick input to min/max of each joint
        """
        lookup = self.brushless_motors if brushless else self.brushed_motors
        min_velocity = lookup[joint_name]["min_velocity"]
        max_velocity = lookup[joint_name]["max_velocity"]
        return self.remap(input, -1, 1, min_velocity, max_velocity)

    # def publish_ik(self, axes: list[float], buttons: list[float]) -> None:
    #     ee_in_map = SE3.from_tf_tree(TF_BUFFER, "base_link", "arm_d_link")
    #
    #     ee_in_map.position[0] += self.ik_names["x"] * self.filter_xbox_axis(axes[self.xbox_mappings["left_js_x"]])
    #     ee_in_map.position[1] += self.ik_names["y"] * self.filter_xbox_axis(axes[self.xbox_mappings["left_js_y"]])
    #     ee_in_map.position[2] += self.ik_names["z"] * self.filter_xbox_button(buttons, "right_trigger", "left_trigger")
    #
    #     self.send(
    #         text_data=json.dumps(
    #             {
    #                 "type": "ik",
    #                 "target": {
    #                     "position": ee_in_map.position.tolist(),
    #                     "quaternion": ee_in_map.rotation.quaternion.tolist(),
    #                 },
    #             }
    #         )
    #     )
    #
    #     self.arm_ik_pub.publish(
    #         IK(
    #             target=PoseStamped(
    #                 header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
    #                 pose=Pose(
    #                     position=Point(*ee_in_map.position),
    #                     orientation=Quaternion(*ee_in_map.rotation.quaternion),
    #                 ),
    #             )
    #         )
    #     )

    def publish_position(self, type, names, positions):
        position_cmd = Position(
            names=names,
            positions=positions,
        )
        if type == "arm_values":
            self.arm_position_cmd_pub.publish(position_cmd)
        elif type == "sa_arm_values":
            self.sa_position_cmd_pub.publish(position_cmd)

    def publish_velocity(self, type, names, axes, buttons):
        left_trigger = self.filter_xbox_axis(axes[self.xbox_mappings["left_trigger"]])
        right_trigger = self.filter_xbox_axis(axes[self.xbox_mappings["right_trigger"]])
        velocity_cmd = Velocity()
        velocity_cmd.names = names
        if type == "sa_arm_values":
            velocity_cmd.velocities = [
                self.to_velocity(self.filter_xbox_axis(axes[self.sa_config["sa_x"]["xbox_index"]]), "sa_x", False),
                self.to_velocity(self.filter_xbox_axis(axes[self.sa_config["sa_y"]["xbox_index"]]), "sa_y", False),
                self.to_velocity(self.filter_xbox_axis(axes[self.sa_config["sa_z"]["xbox_index"]]), "sa_z", True),
                self.sa_config["sampler"]["multiplier"] * (right_trigger - left_trigger),
                self.sa_config["sensor_actuator"]["multiplier"]
                * self.filter_xbox_button(buttons, "right_bumper", "left_bumper"),
            ]
            self.sa_velocity_cmd_pub.publish(velocity_cmd)
        elif type == "arm_values":
            velocity_cmd.velocities = [
                self.to_velocity(self.filter_xbox_axis(axes[self.ra_config["joint_a"]["xbox_index"]]), "joint_a"),
                self.to_velocity(
                    self.filter_xbox_axis(axes[self.ra_config["joint_b"]["xbox_index"]]), "joint_b", False
                ),
                self.to_velocity(self.filter_xbox_axis(axes[self.ra_config["joint_c"]["xbox_index"]]), "joint_c"),
                self.to_velocity(
                    self.filter_xbox_axis(axes[self.ra_config["joint_de_pitch"]["xbox_index"]]), "joint_de_0"
                ),
                self.to_velocity(
                    self.filter_xbox_axis(axes[self.ra_config["joint_de_roll"]["xbox_index"]]), "joint_de_1"
                ),
                self.ra_config["allen_key"]["multiplier"] * self.filter_xbox_button(buttons, "y", "a"),
                self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(buttons, "b", "x"),
            ]
            self.arm_velocity_cmd_pub.publish(velocity_cmd)

    def publish_throttle(self, type, names, axes, buttons):
        left_trigger = self.filter_xbox_axis(buttons[self.xbox_mappings["left_trigger"]])
        right_trigger = self.filter_xbox_axis(buttons[self.xbox_mappings["right_trigger"]])
        throttle_cmd = Throttle()
        throttle_cmd.names = names
        if type == "arm_values":
            throttle_cmd.throttles = [
                self.ra_config["joint_a"]["multiplier"] * self.filter_xbox_axis(axes[self.xbox_mappings["left_x"]]),
                self.ra_config["joint_b"]["multiplier"] * self.filter_xbox_axis(axes[self.xbox_mappings["left_y"]]),
                self.ra_config["joint_c"]["multiplier"]
                * quadratic(self.filter_xbox_axis(axes[self.xbox_mappings["right_y"]])),
                self.ra_config["joint_de_pitch"]["multiplier"]
                * self.filter_xbox_axis(
                    buttons[self.xbox_mappings["right_trigger"]] - buttons[self.xbox_mappings["left_trigger"]]
                ),
                self.ra_config["joint_de_roll"]["multiplier"]
                * self.filter_xbox_axis(
                    buttons[self.xbox_mappings["right_bumper"]] - buttons[self.xbox_mappings["left_bumper"]]
                ),
                self.ra_config["allen_key"]["multiplier"] * self.filter_xbox_button(buttons, "y", "a"),
                self.ra_config["gripper"]["multiplier"] * self.filter_xbox_button(buttons, "b", "x"),
            ]
            self.arm_throttle_cmd_pub.publish(throttle_cmd)
        elif type == "sa_arm_values":
            throttle_cmd.throttles = [
                self.filter_xbox_axis(axes[self.sa_config["sa_x"]["xbox_index"]])
                * self.sa_config["sa_x"]["multiplier"],
                self.filter_xbox_axis(axes[self.sa_config["sa_y"]["xbox_index"]])
                * self.sa_config["sa_y"]["multiplier"],
                self.filter_xbox_axis(axes[self.sa_config["sa_z"]["xbox_index"]])
                * self.sa_config["sa_z"]["multiplier"],
                (right_trigger - left_trigger) * self.sa_config["sampler"]["multiplier"],
                self.filter_xbox_button(buttons, "right_bumper", "left_bumper")
                * self.sa_config["sensor_actuator"]["multiplier"],
            ]
            self.sa_throttle_cmd_pub.publish(throttle_cmd)

    def handle_cache(self, mode, input, positions):
        if mode == "position":
            position_cmd = Position(
                names=["cache"],
                positions=positions,
            )
            self.cache_position_cmd_pub.publish(position_cmd)
        elif mode == "velocity":
            velocity_cmd = Velocity()
            velocity_cmd.names = ["cache"]
            velocity_cmd.velocities = [self.sa_config["cache"]["multiplier"] * input]
            self.cache_velocity_cmd_pub.publish(velocity_cmd)
        else:
            throttle_cmd = Throttle()
            throttle_cmd.names = ["cache"]
            throttle_cmd.throttles = [self.sa_config["cache"]["multiplier"] * input]
            self.cache_throttle_cmd_pub.publish(throttle_cmd)

    def handle_controls_message(self, msg):
        names = ["joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"]
        if msg["type"] == "sa_arm_values":
            names = ["sa_x", "sa_y", "sa_z", "sampler", "sensor_actuator"]

        if msg["type"] == "cache_values":
            self.handle_cache(mode=msg["arm_mode"], input=msg["input"], positions=msg["positions"])

        # elif msg["buttons"][self.xbox_mappings["home"]] > 0.5:
        #     rospy.loginfo("Homing DE")
        #
        #     client = actionlib.SimpleActionClient("arm_action", ArmActionAction)
        #     if client.wait_for_server(timeout=rospy.Duration(1)):
        #         goal = ArmActionGoal(name="de_home")
        #         client.send_goal(goal)
        #
        #         client.wait_for_result()
        #     else:
        #         rospy.logwarn("Arm action server not available")
        # else:

        def filter_axis(
            name: str, deadzone_threshold: float = 0.05, apply_quadratic: bool = False, scale: float = 1.0
        ) -> float:
            return self.filter_axis(
                msg["axes"],
                self.xbox_mappings,
                name,
                deadzone_threshold,
                apply_quadratic,
                scale * self.ra_config[name]["multiplier"],
            )

        self.xbox_twist = np.zeros(2)
        self.xbox_twist -= np.array(
            [
                self.filter_xbox_axis(msg["axes"][self.xbox_mappings["left_y"]]),
                self.filter_xbox_axis(msg["axes"][self.xbox_mappings["right_x"]]),
            ]
        )
        self.send_twist()

        # if msg["arm_mode"] == "ik":
        #     self.publish_ik(msg["axes"], msg["buttons"])

        if msg["arm_mode"] == "position":
            self.publish_position(type=msg["type"], names=names, positions=msg["positions"])

        elif msg["arm_mode"] == "velocity":
            self.publish_velocity(type=msg["type"], names=names, axes=msg["axes"], buttons=msg["buttons"])

        elif msg["arm_mode"] == "throttle":
            self.publish_throttle(type=msg["type"], names=names, axes=msg["axes"], buttons=msg["buttons"])

    def handle_joystick_message(self, msg):
        # Tiny deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg["axes"][self.mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1].
        # Negative sign required to drive forward by default instead of backward.
        # (-1*dampen) because the top of the dampen switch is -1.
        dampen = -1 * ((-1 * dampen) + 1) / 2

        def filter_axis(
            name: str, deadzone_threshold: float = 0.05, apply_quadratic: bool = False, scale: float = 1.0
        ) -> float:
            return self.filter_axis(
                msg["axes"],
                self.joystick_mappings,
                name,
                deadzone_threshold,
                apply_quadratic,
                scale * self.drive_config[name]["multiplier"],
            )

        self.joystick_twist = np.zeros(2)
        self.joystick_twist += np.array(
            [
                filter_axis("forward_back", 0.02, True, self.max_wheel_speed * dampen),
                # Note(quintin): I prefer using solely the twist axis for turning...
                filter_axis("twist", 0.03, True, self.max_angular_speed * dampen),
            ]
        )
        self.joystick_twist -= np.array(
            [
                filter_axis("tilt", 0.5, scale=0.1),
                filter_axis("pan", 0.5, scale=0.1),
            ]
        )
        self.send_twist()

        self.send(
            text_data=json.dumps(
                {
                    "type": "joystick",
                    "left_right": msg["axes"][self.joystick_mappings["left_right"]],
                    "forward_back": msg["axes"][self.joystick_mappings["forward_back"]],
                    "twist": msg["axes"][self.joystick_mappings["twist"]],
                    "dampen": msg["axes"][self.joystick_mappings["dampen"]],
                    "pan": msg["axes"][self.joystick_mappings["pan"]],
                    "tilt": msg["axes"][self.joystick_mappings["tilt"]],
                }
            )
        )

    def pdb_callback(self, msg):
        self.send(text_data=json.dumps({"type": "pdb", "temperatures": msg.temperatures, "currents": msg.currents}))

    def arm_controller_callback(self, msg):
        hits = []
        for n in msg.limit_hit:
            temp = []
            for i in range(4):
                temp.append((1 if n & (1 << i) != 0 else 0))
            hits.append(temp)
        self.send(
            text_data=json.dumps(
                {"type": "arm_moteus", "name": msg.name, "state": msg.state, "error": msg.error, "limit_hit": hits}
            )
        )

    def sa_joint_callback(self, msg):
        names = msg.name
        z = msg.position[names.index("sa_z")]
        self.send(text_data=json.dumps({"type": "sa_z", "sa_z": z}))

    def drive_controller_callback(self, msg):
        hits = []
        for n in msg.limit_hit:
            temp = []
            for i in range(4):
                temp.append((1 if n & (1 << i) != 0 else 0))
            hits.append(temp)
        self.send(
            text_data=json.dumps(
                {"type": "drive_moteus", "name": msg.name, "state": msg.state, "error": msg.error, "limit_hit": hits}
            )
        )

    def enable_laser_callback(self, msg):
        try:
            result = self.laser_service(data=msg["data"])
            self.send(text_data=json.dumps({"type": "laser_service", "success": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def limit_switch(self, msg):
        fail = []
        if msg["name"] == "all_ra":
            for joint in self.RA_NAMES:
                name = "arm_enable_limit_switch_" + joint
                self.limit_switch_service = rospy.ServiceProxy(name, SetBool)
                try:
                    result = self.limit_switch_service(data=msg["data"])
                    if not result.success:
                        fail.append(joint)
                except rospy.ServiceException as e:
                    print(f"Service call failed: {e}")
        else:
            self.limit_switch_service = rospy.ServiceProxy(msg["name"], SetBool)
            try:
                result = self.limit_switch_service(data=msg["data"])
                if not result.success:
                    fail.append(joint)
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

    def enable_white_leds_callback(self, msg):
        for i in range(0, 3):
            white_led_name = f"science_enable_white_led_{i}"
            led_srv = rospy.ServiceProxy(white_led_name, SetBool)
            try:
                result = led_srv(data=msg["data"])
                self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def enable_uv_leds_callback(self, msg):
        for i in range(0, 3):
            uv_led_name = f"science_enable_uv_led_{i}"
            led_srv = rospy.ServiceProxy(uv_led_name, SetBool)
            try:
                result = led_srv(data=msg["data"])
                self.send(text_data=json.dumps({"type": "toggle_uv", "result": result.success}))
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

    def enable_device_callback(self, msg):
        try:
            result = self.calibrate_service()
            self.send(text_data=json.dumps({"type": "calibrate_service", "result": result.success}))
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def calibrate_motors(self, msg):
        fail = []  # if any calibration fails, add joint name to a list to return
        if msg["topic"] == "all_ra":
            for joint in self.RA_NAMES:
                self.calibrate_service = rospy.ServiceProxy("arm_calibrate_" + joint, Trigger)
                try:
                    result = self.calibrate_service()
                    if not result.success:
                        fail.append(joint)
                except rospy.ServiceException as e:
                    print(f"Service call failed: {e}")
        else:
            self.calibrate_service = rospy.ServiceProxy(msg["topic"], Trigger)
            try:
                result = self.calibrate_service()
                if not result.success:
                    fail = msg["topic"]
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

        self.send(text_data=json.dumps({"type": "calibrate_motors", "result": fail}))

    def arm_adjust(self, msg):
        try:
            arm_adjust_srv = rospy.ServiceProxy(msg["name"] + "_adjust", AdjustMotor)
            result = arm_adjust_srv(name=msg["name"], value=msg["value"])
            self.send(text_data=json.dumps({"type": "arm_adjust", "success": result.success}))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def reset_gimbal(self):
        try:
            adjust_srv = rospy.ServiceProxy("mast_gimbal_z_adjust", AdjustMotor)
            result = adjust_srv(name="mast_gimbal_z", value=0)
            self.send(text_data=json.dumps({"type": "arm_adjust", "success": result.success}))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def disable_auton_led(self):
        message = String()
        message.data = "off"
        self.led_pub.publish(message)

    def drive_status_callback(self, msg):
        msg.joint_states.position = [x * self.wheel_radius for x in msg.joint_states.position]
        msg.joint_states.velocity = [x * self.wheel_radius for x in msg.joint_states.velocity]
        self.send(
            text_data=json.dumps(
                {
                    "type": "drive_status",
                    "name": msg.name,
                    "position": msg.joint_states.position,
                    "velocity": msg.joint_states.velocity,
                    "effort": msg.joint_states.effort,
                    "state": msg.moteus_states.state,
                    "error": msg.moteus_states.error,
                }
            )
        )

    def cmd_vel_callback(self, msg):
        self.send(text_data=json.dumps({"type": "cmd_vel", "linear_x": msg.linear.x, "angular_z": msg.angular.z}))

    def gps_fix_callback(self, msg):
        self.send(
            text_data=json.dumps(
                {"type": "nav_sat_fix", "latitude": msg.latitude, "longitude": msg.longitude, "altitude": msg.altitude}
            )
        )

    def send_auton_command(self, msg):
        self.enable_auton(
            msg["is_enabled"],
            [
                GPSWaypoint(
                    waypoint["tag_id"],
                    waypoint["latitude_degrees"],
                    waypoint["longitude_degrees"],
                    WaypointType(waypoint["type"]),
                )
                for waypoint in msg["waypoints"]
            ],
        )

    def send_teleop_enabled(self, msg):
        rospy.wait_for_service("enable_teleop")
        try:
            enable_teleop = rospy.ServiceProxy("enable_teleop", SetBool)
            enable_teleop(msg["data"])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def led_callback(self, msg):
        self.send(
            text_data=json.dumps(
                {"type": "led", "red": msg.red, "green": msg.green, "blue": msg.blue, "is_blinking": msg.is_blinking}
            )
        )

    def nav_state_callback(self, msg):
        self.send(text_data=json.dumps({"type": "nav_state", "state": msg.state}))

    def sa_temp_data_callback(self, msg):
        self.send(text_data=json.dumps({"type": "temp_data", "temp_data": msg.temperature}))

    def sa_humidity_data_callback(self, msg):
        self.send(text_data=json.dumps({"type": "relative_humidity", "humidity_data": msg.relative_humidity}))

    def ish_thermistor_data_callback(self, msg):
        temps = [x.temperature for x in msg.temps]
        self.send(text_data=json.dumps({"type": "thermistor", "temps": temps}))

    def bearing(self):
        base_link_in_map = SE3.from_tf_tree(TF_BUFFER, "map", "base_link")
        self.send(
            text_data=json.dumps(
                {
                    "type": "bearing",
                    "rotation": base_link_in_map.rotation.quaternion.tolist(),
                }
            )
        )

    def mast_gimbal(self, msg):
        pwr = rospy.get_param("teleop/mast_gimbal_power")
        rot_pwr = msg["throttles"][0] * pwr["rotation_pwr"]
        up_down_pwr = msg["throttles"][1] * pwr["up_down_pwr"]
        self.mast_gimbal_pub.publish(Throttle(["mast_gimbal_y", "mast_gimbal_z"], [rot_pwr, up_down_pwr]))

    def change_cameras(self, msg):
        try:
            camera_cmd = CameraCmd(msg["device"], msg["resolution"])
            result = self.change_cameras_srv(primary=msg["primary"], camera_cmd=camera_cmd)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def send_res_streams(self):
        res = rospy.get_param("cameras/max_num_resolutions")
        streams = rospy.get_param("cameras/max_streams")
        self.send(text_data=json.dumps({"type": "max_resolution", "res": res}))
        self.send(text_data=json.dumps({"type": "max_streams", "streams": streams}))

    def capture_panorama(self) -> None:
        rospy.logerr("Capturing panorama")
        goal = CapturePanoramaGoal()
        goal.angle = pi / 2

        def feedback_cb(feedback: CapturePanoramaGoal) -> None:
            self.send(text_data=json.dumps({"type": "pano_feedback", "percent": feedback.percent_done}))

        self.pano_client.send_goal(goal, feedback_cb=feedback_cb)
        finished = self.pano_client.wait_for_result(timeout=rospy.Duration(30))  # timeouts after 30 seconds
        if finished:
            rospy.logerr("Finished!")
            image = self.pano_client.get_result().panorama
            self.image_callback(image)
        else:
            rospy.logerr("CapturePanorama took too long!")

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            # Convert the image to OpenCV standard format
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("Could not convert image message to OpenCV image: " + str(e))
            return

        # Save the image to a file (you could change 'png' to 'jpg' or other formats)
        image_filename = "~/Downloads/panorama.png"
        try:
            cv2.imwrite(os.path.expanduser(image_filename), cv_image)
            rospy.logerr("Saved image to {}".format(image_filename))
        except Exception as e:
            rospy.logerr("Could not save image: " + str(e))

    def heater_enable_service(self, msg):
        try:
            heater = msg["heater"]
            science_enable = rospy.ServiceProxy("science_enable_heater_" + heater, SetBool)
            result = science_enable(data=msg["enabled"])
        except rospy.ServiceException as e:
            print(f"Service init failed: {e}")

    def ish_heater_state_callback(self, msg):
        self.send(text_data=json.dumps({"type": "heater_states", "states": msg.state}))

    def auto_shutoff_toggle(self, msg):
        try:
            rospy.logerr(msg)
            result = self.heater_auto_shutoff_srv(data=msg["shutoff"])
            self.send(text_data=json.dumps({"type": "auto_shutoff", "success": result.success}))
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def send_center(self):
        lat = rospy.get_param("gps_linearization/reference_point_latitude")
        long = rospy.get_param("gps_linearization/reference_point_longitude")
        self.send(text_data=json.dumps({"type": "center_map", "latitude": lat, "longitude": long}))

    def save_auton_waypoint_list(self, msg):
        AutonWaypoint.objects.all().delete()
        waypoints = []
        for w in msg["data"]:
            waypoints.append(
                AutonWaypoint(tag_id=w["id"], type=w["type"], latitude=w["lat"], longitude=w["lon"], name=w["name"])
            )
        AutonWaypoint.objects.bulk_create(waypoints)
        self.send(text_data=json.dumps({"type": "save_auton_waypoint_list", "success": True}))
        # Print out all of the waypoints
        for w in AutonWaypoint.objects.all():
            rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))

    def get_auton_waypoint_list(self, msg):
        waypoints = []
        for w in AutonWaypoint.objects.all():
            waypoints.append({"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type})
        self.send(text_data=json.dumps({"type": "get_auton_waypoint_list", "data": waypoints}))

    def save_basic_waypoint_list(self, msg):
        BasicWaypoint.objects.all().delete()
        waypoints = []
        for w in msg["data"]:
            waypoints.append(BasicWaypoint(drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]))
        BasicWaypoint.objects.bulk_create(waypoints)
        self.send(text_data=json.dumps({"type": "save_basic_waypoint_list", "success": True}))
        # Print out all of the waypoints
        for w in BasicWaypoint.objects.all():
            rospy.loginfo(str(w.name) + " " + str(w.latitude) + " " + str(w.longitude))

    def get_basic_waypoint_list(self, msg):
        waypoints = []
        for w in BasicWaypoint.objects.all():
            waypoints.append({"name": w.name, "drone": w.drone, "lat": w.latitude, "lon": w.longitude})
        self.send(text_data=json.dumps({"type": "get_basic_waypoint_list", "data": waypoints}))

    def imu_calibration_callback(self, msg) -> None:
        self.send(text_data=json.dumps({"type": "calibration_status", "system_calibration": msg.system_calibration}))

    def flight_attitude_listener(self):
        # threshold that must be exceeded to send JSON message
        threshold = 0.1
        map_to_baselink = SE3()

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                tf_msg = SE3.from_tf_tree(TF_BUFFER, "map", "base_link")

                if tf_msg.is_approx(map_to_baselink, threshold):
                    rate.sleep()
                    continue
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rate.sleep()
                continue

            map_to_baselink = tf_msg
            rotation = map_to_baselink.rotation
            euler = euler_from_quaternion(rotation.quaternion)
            pitch = euler[0] * 180 / pi
            roll = euler[1] * 180 / pi

            self.send(text_data=json.dumps({"type": "flight_attitude", "pitch": pitch, "roll": roll}))

            rate.sleep()

    def arm_joint_callback(self, msg: JointState) -> None:
        # Set non-finite values to zero
        msg.position = [x if isfinite(x) else 0 for x in msg.position]
        self.send(text_data=json.dumps({"type": "fk", "positions": msg.position}))

    def science_spectral_callback(self, msg):
        self.send(
            text_data=json.dumps({"type": "spectral_data", "site": msg.site, "data": msg.data, "error": msg.error})
        )

    def download_csv(self, msg):
        username = os.getenv("USERNAME", "-1")

        now = datetime.now(pytz.timezone("US/Eastern"))
        current_time = now.strftime("%m-%d-%Y_%H:%M:%S")
        spectral_data = msg["data"]

        site_names = ["0", "1", "2"]
        index = 0

        # add site letter in front of data
        for site_data in spectral_data:
            site_data.insert(0, f"Site {site_names[index]}")
            index = index + 1

        time_row = ["Time", current_time]
        spectral_data.insert(0, time_row)

        if username == "-1":
            rospy.logerr("username not found")

        with open(os.path.join(f"/home/{username}/Downloads/spectral_data_{current_time}.csv"), "w") as f:
            csv_writer = csv.writer(f)
            csv_writer.writerows(spectral_data)
