import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import time
import math
import json
import yaml
import os
from .search import generate_lawnmower_waypoints
from .px4_gateway import PX4Gateway
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand


class MissionNode(Node):
    def __init__(self, config):
        super().__init__('mission_node')
        self.config = config
        self.gateway = PX4Gateway(config)

        # ---------- 任务模式与状态 ----------
        self.task_mode = "TAKEOFF"        # 启动后自动起飞
        self.target_id = 0
        self.prev_mode = "IDLE"
        self.target_positions = {}
        self.search_state = {}
        self.switch_goal = None

        # ---------- 命令与偏移缓存 ----------
        self.nav_cmd = None
        self.offset_cmd = None

        # ---------- 起飞相关 ----------
        self.armed = False
        self.takeoff_complete = False
        self.prestream_start = None
        self.prestream_duration = 2.0
        self.takeoff_altitude = -config.get('takeoff_altitude', 5.0)

        # ---------- 返航相关 ----------
        self._rtl_requested = False
        self._rtl_start_time = 0.0
        self.RTL_TIMEOUT = 10.0

        # ---------- 航点任务相关（新增） ----------
        self.waypoints = config.get('waypoints', [])
        self.wp_mission_idx = 0
        self.wp_mission_state = {}
        self.wp_hold_start = None
        self.wp_armed = False
        self.wp_prestream_start = None

        # ---------- 订阅 ----------
        nav_topic = config['topics'].get('navigation_command', '/navigation_command')
        offset_topic = config['topics'].get('control_target_offset', '/control_target_offset')
        self.nav_sub = self.create_subscription(String, nav_topic, self.nav_callback, 10)
        self.offset_sub = self.create_subscription(String, offset_topic, self.offset_callback, 10)

        # ---------- 定时器 ----------
        self.create_timer(1.0 / config['control_frequency'], self.control_loop)

        # ---------- 发布者 ----------
        self.mission_state_pub = self.create_publisher(String, '/tracking/mission_state', 10)

        self.get_logger().info("MissionNode initialized, auto-takeoff will start.")

    # ========== 回调函数 ==========
    def nav_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.nav_cmd = data
            self.get_logger().info(f"Nav cmd: {data.get('command')}")
        except Exception as e:
            self.get_logger().error(f"Nav parse error: {e}")

    def offset_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.offset_cmd = data
            if data.get('enable', False):
                self.get_logger().info(
                    f"Tracking ON: ID={data['target_id']}, "
                    f"offset=({data['offset_x']}, {data['offset_y']})"
                )
                self.prev_mode = self.task_mode
                self.task_mode = "TRACKING"
                self.target_id = data['target_id']
            else:
                self.get_logger().info("Tracking OFF")
                if self.task_mode == "TRACKING":
                    self.task_mode = self.prev_mode if self.prev_mode != "TRACKING" else "IDLE"
                self.offset_cmd = None
        except Exception as e:
            self.get_logger().error(f"Offset parse error: {e}")

    # ========== 主控制循环 ==========
    def control_loop(self):
        # 1. 处理导航命令（仅起飞完成后）
        if self.nav_cmd is not None and self.takeoff_complete:
            self.process_nav_command(self.nav_cmd)
            self.nav_cmd = None

        # 2. 如果起飞未完成，执行起飞
        if not self.takeoff_complete:
            self.execute_takeoff()
            return

        # 3. 手动接管检测（仅当已起飞）
        if self.gateway.status is not None:
            nav = self.gateway.status.nav_state
            if nav not in (14, 4, 5, 18):
                self.get_logger().warn(f"Manual takeover (nav_state={nav})")
                self.publish_state("MANUAL")
                return

        # 4. 执行当前模式
        if self.task_mode == "IDLE":
            self.execute_idle()
        elif self.task_mode == "SEARCHING":
            self.execute_search()
        elif self.task_mode == "TRACKING":
            self.execute_tracking()
        elif self.task_mode == "SWITCHING":
            self.execute_switching()
        elif self.task_mode == "RETURNING":
            self.execute_returning()
        elif self.task_mode == "WAYPOINT_MISSION":
            self.execute_waypoint_mission()
        else:
            self.get_logger().warn(f"Unknown mode: {self.task_mode}")
            self.task_mode = "IDLE"

    # ========== 命令处理 ==========
    def process_nav_command(self, cmd):
        command = cmd.get('command')
        if command == 'start_search':
            target_id = cmd.get('target_id')
            if target_id is None:
                self.get_logger().error("start_search missing target_id")
                return
            self.target_id = target_id
            self.get_logger().info(f"Start search for ID {target_id}")
            self.init_search(target_id)
            self.task_mode = "SEARCHING"

        elif command == 'switch_target':
            observed = cmd.get('observed_target_id')
            requested = cmd.get('requested_target_id')
            self.get_logger().info(f"Switch from {observed} to {requested}")
            self.target_id = requested
            self.task_mode = "SWITCHING"
            self.init_switch(requested, use_recorded=False)

        elif command == 'wrong_target':
            observed = cmd.get('observed_target_id')
            requested = cmd.get('requested_target_id')
            self.get_logger().info(f"Wrong target: at {observed}, go to {requested}")
            self.target_id = requested
            self.task_mode = "SWITCHING"
            self.init_switch(requested, use_recorded=True)

        elif command == 'return_home':
            self.get_logger().info("Return home command received")
            self.task_mode = "RETURNING"
            self.init_return_home()

        elif command == 'start_waypoint_mission':
            if not self.takeoff_complete:
                self.get_logger().error("Cannot start waypoint mission before takeoff complete")
                return
            self.wp_mission_idx = 0
            self.wp_mission_state = {}
            self.wp_hold_start = None
            self.wp_armed = False
            self.wp_prestream_start = None
            self.task_mode = "WAYPOINT_MISSION"
            self.get_logger().info("Waypoint mission started")

        else:
            self.get_logger().error(f"Unknown command: {command}")

    # ========== 起飞 ==========
    def execute_takeoff(self):
        #print("11111111111111111111111")
        altitude = self.takeoff_altitude
        if not self.armed:
            if self.gateway.status is None:
                return
            if self.prestream_start is None:
                self.prestream_start = time.time()
            self.gateway.publish_offboard_heartbeat()
            self.gateway.publish_trajectory_setpoint(0.0, 0.0, altitude)
            self.get_logger().info(f"Pre-stream setpoint: (0, 0, {altitude})")
            if time.time() - self.prestream_start > self.prestream_duration:
                self.gateway.request_offboard_mode()
                self.gateway.arm()
            if (self.gateway.status.arming_state == 2 and
                self.gateway.status.nav_state == 14):
                self.armed = True
                self.get_logger().info("Armed and Offboard. Starting takeoff.")
                self.prestream_start = None
            return

        pos = self.gateway.local_pos
        if pos is None:
            return
        self.gateway.publish_offboard_heartbeat()
        self.gateway.publish_trajectory_setpoint(0.0, 0.0, altitude)
        self.get_logger().info(f"Takeoff setpoint: (0, 0, {altitude}), current z: {pos.z:.2f}")
        if abs(pos.z - altitude) < self.config.get('vertical_tolerance', 0.6):
            self.get_logger().info("Takeoff altitude reached, switching to IDLE.")
            self.takeoff_complete = True
            self.task_mode = "IDLE"

    # ========== IDLE ==========
    def execute_idle(self):
        self.gateway.publish_offboard_heartbeat()
        pos = self.gateway.local_pos
        if pos is not None:
            self.gateway.publish_trajectory_setpoint(pos.x, pos.y, pos.z)
        self.publish_state("IDLE")

    # ========== 搜索 ==========
    def init_search(self, target_id):
        area = self.config['target_areas'].get(target_id)
        if not area:
            self.get_logger().error(f"No search area for ID {target_id}")
            self.task_mode = "IDLE"
            return
        wps = generate_lawnmower_waypoints(
            area['center_north'],
            area['center_east'],
            area['length_north'],
            area['width_east'],
            area['spacing'],
            area['altitude']
        )
        self.search_state = {
            'wps': wps,
            'idx': 0,
            'start_time': time.time(),
            'timeout': area.get('timeout', 60.0),
            'track_alt_offset': area.get('track_alt_offset', 2.0)
        }
        self.get_logger().info(f"Generated {len(wps)} search waypoints for ID {target_id}")

    def execute_search(self):
        state = self.search_state
        if not state:
            self.get_logger().warn("Search state empty, aborting")
            self.task_mode = "IDLE"
            return

        if time.time() - state['start_time'] > state['timeout']:
            self.get_logger().info("Search timeout, switching to IDLE")
            self.task_mode = "IDLE"
            self.search_state = {}
            return

        wps = state['wps']
        idx = state['idx']
        if idx < len(wps):
            wp = wps[idx]
            self.gateway.publish_offboard_heartbeat()
            self.gateway.publish_trajectory_setpoint(*wp)
            pos = self.gateway.local_pos
            if pos is not None:
                dist = math.sqrt((pos.x - wp[0])**2 + (pos.y - wp[1])**2)
                if dist < self.config['position_tolerance']:
                    state['idx'] += 1
        else:
            state['idx'] = 0
        self.publish_state("SEARCHING")

    # ========== 跟踪 ==========
    def execute_tracking(self):
        if self.offset_cmd is None or not self.offset_cmd.get('enable', False):
            self.get_logger().warn("Tracking enabled but offset invalid")
            self.task_mode = "IDLE"
            return

        pos = self.gateway.local_pos
        if pos is None:
            return

        # 直接从偏移量获取东/北向偏移（已经是 NED 系）
        ox = self.offset_cmd['offset_x']   # 东向
        oy = self.offset_cmd['offset_y']   # 北向

        # 目标位置（水平）
        target_n = pos.x + oy
        target_e = pos.y + ox
        self.get_logger().info(f"📊 ox={ox:.3f}, oy={oy:.3f} -> target_n={target_n:.2f}, target_e={target_e:.2f}")
        # 固定跟踪高度（NED 下为负值）
        target_d = -5.0   # 5 米

        # 调试日志
        self.get_logger().info(f"🔍 TRACKING: pos.z={pos.z:.2f}, target_d={target_d:.2f}")

        self.gateway.publish_offboard_heartbeat()
        self.gateway.publish_trajectory_setpoint(target_n, target_e, target_d)

        self.target_positions[self.target_id] = (target_n, target_e, target_d)
        self.publish_state("TRACKING")
    # ========== 切换 ==========
    def init_switch(self, target_id, use_recorded=False):
        if use_recorded and target_id in self.target_positions:
            n, e, d = self.target_positions[target_id]
        else:
            area = self.config['target_areas'].get(target_id)
            if not area:
                self.get_logger().error(f"Target area {target_id} not defined")
                self.task_mode = "IDLE"
                return
            n = area['center_north']
            e = area['center_east']
            d = -area['altitude']
        self.switch_goal = (n, e, d)
        self.get_logger().info(f"Switching to target {target_id} at ({n:.1f}, {e:.1f}, {d:.1f})")

    def execute_switching(self):
        if self.switch_goal is None:
            self.get_logger().warn("Switch goal not set")
            self.task_mode = "IDLE"
            return

        goal_n, goal_e, goal_d = self.switch_goal
        self.gateway.publish_offboard_heartbeat()
        self.gateway.publish_trajectory_setpoint(goal_n, goal_e, goal_d)

        pos = self.gateway.local_pos
        if pos is not None:
            dist = math.sqrt((pos.x - goal_n)**2 + (pos.y - goal_e)**2)
            if dist < self.config['position_tolerance'] and abs(pos.z - goal_d) < self.config.get('vertical_tolerance', 0.6):
                self.get_logger().info("Reached target area, waiting for visual tracking")
                self.task_mode = "IDLE"
                self.switch_goal = None
                self.publish_state("SWITCH_COMPLETE")

    # ========== 返航 ==========
    def init_return_home(self):
        if not self._rtl_requested:
            # 直接发送 RTL 导航命令（VEHICLE_CMD_NAV_RETURN_TO_LAUNCH）
            self.gateway.send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, 0.0, 0.0)
            self._rtl_requested = True
            self._rtl_start_time = time.time()
            self.get_logger().info("RTL command sent (NAV_RETURN_TO_LAUNCH). Waiting for transition.")

    def execute_returning(self):
        if self.gateway.status is None:
            return

        nav = self.gateway.status.nav_state
        arming = self.gateway.status.arming_state
        self.get_logger().info(f"RTL status: nav_state={nav}, arming_state={arming}")

        # 成功进入 RTL 或已上锁 → 完成
        if nav == 4 or arming == 0:
            self.get_logger().info("RTL engaged or disarmed. Mission finished.")
            self.task_mode = "IDLE"
            self.publish_state("RETURNED")
            self._rtl_requested = False
            return

        # 超时处理
        if time.time() - self._rtl_start_time > self.RTL_TIMEOUT:
            self.get_logger().error("RTL timeout! Switching to POSITION mode as fallback.")
            self.gateway.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 0.0)  # 切到 POSITION
            self.task_mode = "IDLE"
            self.publish_state("RTL_TIMEOUT")
            self._rtl_requested = False
            return

        # 等待期间，继续发送 Offboard 心跳和悬停设定点，防止故障保护
        self.gateway.publish_offboard_heartbeat()
        pos = self.gateway.local_pos
        if pos is not None:
            self.gateway.publish_trajectory_setpoint(pos.x, pos.y, pos.z)

    # ========== 航点任务执行（新增） ==========
    def execute_waypoint_mission(self):
        if not self.waypoints or self.wp_mission_idx >= len(self.waypoints):
            self.get_logger().info("Waypoint mission complete.")
            self.task_mode = "IDLE"
            self.publish_state("WAYPOINT_MISSION_COMPLETE")
            return

        wp = self.waypoints[self.wp_mission_idx]
        wp_type = wp.get('type', 'waypoint')
        self.publish_state(f"WP_{wp_type.upper()}_{self.wp_mission_idx}")

        if wp_type == 'takeoff':
            self._wp_takeoff(wp)
        elif wp_type == 'waypoint':
            self._wp_waypoint(wp)
        elif wp_type == 'land':
            self._wp_land(wp)
        elif wp_type == 'rtl':
            self._wp_rtl(wp)
        elif wp_type == 'search':
            self._wp_search(wp)
        else:
            self.get_logger().error(f"Unknown waypoint type: {wp_type}")
            self.wp_mission_idx += 1

    def _wp_takeoff(self, wp):
        altitude = -wp.get('altitude', self.config.get('takeoff_altitude', 5.0))
        hold = wp.get('hold', 0.0)

        if not self.wp_armed:
            if self.gateway.status is None:
                return
            if self.wp_prestream_start is None:
                self.wp_prestream_start = time.time()
            self.gateway.publish_offboard_heartbeat()
            self.gateway.publish_trajectory_setpoint(0.0, 0.0, altitude)
            if time.time() - self.wp_prestream_start > self.prestream_duration:
                self.gateway.request_offboard_mode()
                self.gateway.arm()
            if (self.gateway.status.arming_state == 2 and
                self.gateway.status.nav_state == 14):
                self.wp_armed = True
                self.get_logger().info("WP takeoff: Armed and Offboard")
                self.wp_prestream_start = None
            return

        self.gateway.publish_offboard_heartbeat()
        pos = self.gateway.local_pos
        if pos is None:
            return
        self.gateway.publish_trajectory_setpoint(pos.x, pos.y, altitude)
        if abs(pos.z - altitude) < self.config.get('vertical_tolerance', 0.6):
            if self.wp_hold_start is None:
                self.wp_hold_start = time.time()
            if time.time() - self.wp_hold_start >= hold:
                self.get_logger().info("WP takeoff complete, advancing")
                self.wp_mission_idx += 1
                self.wp_hold_start = None

    def _wp_waypoint(self, wp):
        north = wp['north']
        east = wp['east']
        down = -wp.get('altitude', 5.0)
        hold = wp.get('hold', 0.0)

        self.gateway.publish_offboard_heartbeat()
        self.gateway.publish_trajectory_setpoint(north, east, down)

        pos = self.gateway.local_pos
        if pos is None:
            return
        dist = math.sqrt((pos.x - north)**2 + (pos.y - east)**2)
        if dist < self.config['position_tolerance'] and abs(pos.z - down) < self.config.get('vertical_tolerance', 0.6):
            if self.wp_hold_start is None:
                self.wp_hold_start = time.time()
            if time.time() - self.wp_hold_start >= hold:
                self.wp_mission_idx += 1
                self.wp_hold_start = None
        else:
            self.wp_hold_start = None

    def _wp_land(self, wp):
        if not hasattr(self, '_wp_land_requested'):
            self.gateway.request_land()
            self._wp_land_requested = True
        if self.gateway.status is not None:
            if self.gateway.status.arming_state == 0:
                self.get_logger().info("WP land complete")
                self.wp_mission_idx += 1
                delattr(self, '_wp_land_requested')

    def _wp_rtl(self, wp):
        if not hasattr(self, '_wp_rtl_requested'):
            self.gateway.request_rtl()
            self._wp_rtl_requested = True
        if self.gateway.status is not None:
            if self.gateway.status.nav_state == 4 or self.gateway.status.arming_state == 0:
                self.get_logger().info("WP RTL complete")
                self.wp_mission_idx += 1
                delattr(self, '_wp_rtl_requested')

    def _wp_search(self, wp):
        if 'search_wps' not in self.wp_mission_state:
            center_n = wp.get('center_north', 40.0)
            center_e = wp.get('center_east', 0.0)
            length_n = wp.get('length_north', 30.0)
            width_e = wp.get('width_east', 20.0)
            spacing = wp.get('spacing', 5.0)
            altitude = wp.get('altitude', 8.0)
            self.wp_mission_state['search_wps'] = generate_lawnmower_waypoints(
                center_n, center_e, length_n, width_e, spacing, altitude
            )
            self.wp_mission_state['search_idx'] = 0
            self.wp_mission_state['search_start_time'] = time.time()

        timeout = wp.get('timeout', 60.0)
        if time.time() - self.wp_mission_state['search_start_time'] > timeout:
            self.get_logger().info("WP search timeout, advancing")
            self.wp_mission_idx += 1
            self.wp_mission_state = {}
            return

        wps = self.wp_mission_state['search_wps']
        idx = self.wp_mission_state['search_idx']
        if idx < len(wps):
            wp_ned = wps[idx]
            self.gateway.publish_offboard_heartbeat()
            self.gateway.publish_trajectory_setpoint(*wp_ned)
            pos = self.gateway.local_pos
            if pos is not None:
                dist = math.sqrt((pos.x - wp_ned[0])**2 + (pos.y - wp_ned[1])**2)
                if dist < self.config['position_tolerance']:
                    self.wp_mission_state['search_idx'] = idx + 1
        else:
            self.wp_mission_state['search_idx'] = 0

    # ========== 辅助 ==========
    def publish_state(self, state_str):
        msg = String()
        msg.data = state_str
        self.mission_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    from ament_index_python.packages import get_package_share_directory
    import os, yaml

    try:
        share_dir = get_package_share_directory('uav_tracking_control')
        config_path = os.path.join(share_dir, 'config', 'mission.yaml')
    except Exception:
        config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'mission.yaml')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    node = MissionNode(config)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.gateway)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.gateway.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()