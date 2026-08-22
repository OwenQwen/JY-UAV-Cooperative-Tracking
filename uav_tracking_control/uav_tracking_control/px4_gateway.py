import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PX4Gateway(Node):
    def __init__(self, config):
        super().__init__('px4_gateway')
        self.config = config
        topics = config['topics']

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, topics['fmu_in']['offboard_control_mode'], 10)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, topics['fmu_in']['trajectory_setpoint'], 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, topics['fmu_in']['vehicle_command'], 10)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, topics['fmu_out']['vehicle_local_position'],
            self.local_pos_callback, qos_profile)
        self.status_sub = self.create_subscription(
            VehicleStatus, topics['fmu_out']['vehicle_status'],
            self.status_callback, qos_profile)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, topics['fmu_out']['vehicle_attitude'],
            self.attitude_callback, qos_profile)

        self.local_pos = None
        self.status = None
        self.yaw = 0.0
        self.last_pos_time = time.time()
        self.last_status_time = time.time()
        self.lost_connection = False
        self.data_received = False

        self.create_timer(0.1, self.check_connection)

    def local_pos_callback(self, msg):
        self.local_pos = msg
        self.last_pos_time = time.time()
        self.data_received = True

    def status_callback(self, msg):
        self.status = msg
        self.last_status_time = time.time()
        self.data_received = True

    def attitude_callback(self, msg):
        q = msg.q  # 四元数数组 [w, x, y, z]
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def check_connection(self):
        if not self.data_received:
            return
        now = time.time()
        timeout = self.config.get('lost_connection_timeout', 3.0)
        if (now - self.last_pos_time > timeout or now - self.last_status_time > timeout):
            if not self.lost_connection:
                self.get_logger().error("Lost connection to PX4! Triggering RTL.")
                self.lost_connection = True
                self.request_rtl()
        else:
            self.lost_connection = False

    def is_offboard(self):
        if self.status is None:
            return False
        return self.status.nav_state == 14

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(time.time() * 1e6)
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, north, east, down, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.position = [north, east, down]
        msg.yaw = yaw
        msg.timestamp = int(time.time() * 1e6)
        self.trajectory_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(time.time() * 1e6)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def request_offboard_mode(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def request_rtl(self):
        # 使用 RTL 命令（NAV_RETURN_TO_LAUNCH）而不是模式切换
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0)
        self.get_logger().info("RTL command sent via NAV_RETURN_TO_LAUNCH.")

    def request_land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)