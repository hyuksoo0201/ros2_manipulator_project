# ## task_manager_node.py
# import time
# import rclpy
# import numpy as np
# from rclpy.node import Node
# from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
# from enum import Enum, auto
# from scipy.spatial.transform import Rotation as R

# from mycobot_system_interfaces.action import ObserveMarker, ExecutePnP, ExecuteTask
# from std_msgs.msg import Bool


# # ==================================================
# # Utility (원본 코드 사고방식 유지)
# # ==================================================
# def coords_to_T(coords_mm_deg):
#     x, y, z, rx, ry, rz = coords_mm_deg
#     T = np.eye(4)
#     T[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
#     T[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3
#     return T


# def T_to_coords(T):
#     t_mm = T[:3, 3] * 1e3
#     rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
#     return [
#         float(t_mm[0]), float(t_mm[1]), float(t_mm[2]),
#         float(rpy[0]), float(rpy[1]), float(rpy[2]),
#     ]


# # ==================================================
# # FSM State
# # ==================================================
# class TaskState(Enum):
#     IDLE = auto()
#     REQUEST_VISION = auto()
#     WAIT_VISION = auto()
#     REQUEST_PNP = auto()
#     WAIT_PNP = auto()
#     UPDATE_STACK = auto()
#     DONE = auto()
#     ERROR = auto()


# class TaskManagerNode(Node):
#     def __init__(self):
#         super().__init__('task_manager_node')

#         # =====================================
#         # Assembly state publisher (실시간 스트리밍)
#         # =====================================
#         self.assembly_pub = self.create_publisher(Bool, '/assembly_start', 10)

#         # 재진입 방지 / 상태 변수
#         self._running = False

#         # 🔥 실시간 송신 타이머 (10 Hz)
#         self.assembly_timer = self.create_timer(0.1, self.publish_assembly_state)

#         # Action clients
#         self.vision_client = ActionClient(self, ObserveMarker, '/vision/observe_marker')
#         self.pnp_client = ActionClient(self, ExecutePnP, '/manipulator/execute_pnp_action')

#         # FSM state
#         self.state = TaskState.IDLE
#         self.marker_ids = []
#         self.current_index = 0
#         self.stack_level = 0
#         self.pick_base_target = None

#         # =====================================
#         # Assembly + stacking (🔥 핵심 수정)
#         # =====================================
#         self.assembly_pose = [60, 180, 105, -180, 0, -45]
#         self.stack_dz_mm = 25.0        # 블록 높이
#         self.stack_dz_sign = -1.0      # 위로 쌓일수록 EE는 내려간다 (원본 의미)

#         # Observe parameters
#         self.obs_samples = 12
#         self.obs_timeout = 0.5
#         self.obs_outlier = 0.03

#         # Action server
#         self.action_server = ActionServer(
#             self,
#             ExecuteTask,
#             '/task/execute',
#             execute_callback=self.execute_cb,
#             goal_callback=self.goal_cb,
#             cancel_callback=self.cancel_cb,
#         )

#         self.get_logger().info("🧠 TaskManager ActionServer ready: /task/execute")
#         self.get_logger().info("📣 Streaming /assembly_start at 10 Hz (True=running, False=idle)")

#     # ==================================================
#     # Real-time publisher
#     # ==================================================
#     def publish_assembly_state(self):
#         """
#         실시간 assembly 상태 송신
#         True  = 작업 중
#         False = 대기 상태
#         """
#         msg = Bool()
#         msg.data = bool(self._running)
#         self.assembly_pub.publish(msg)

#     # ==================================================
#     # Reset helpers
#     # ==================================================
#     def reset_fsm(self):
#         self.state = TaskState.IDLE
#         self.marker_ids = []
#         self.current_index = 0
#         self.pick_base_target = None
#         self._running = False
#         self.assembly_pub.publish(Bool(data=False))

#         self.get_logger().info("🔄 FSM reset to IDLE")

#     # ==================================================
#     # Action callbacks
#     # ==================================================
#     def goal_cb(self, goal_request):
#         if self._running:
#             self.get_logger().warn("⚠️ Task already running -> reject new goal")
#             return GoalResponse.REJECT
#         if len(goal_request.marker_ids) == 0:
#             return GoalResponse.REJECT
#         return GoalResponse.ACCEPT

#     def cancel_cb(self, goal_handle):
#         self.get_logger().warn("🛑 Task cancel requested")
#         return CancelResponse.ACCEPT

#     async def execute_cb(self, goal_handle):
#         goal = goal_handle.request
#         feedback = ExecuteTask.Feedback()
#         result = ExecuteTask.Result()

#         self._running = True

#         self.marker_ids = list(goal.marker_ids)
#         self.current_index = 0
#         self.pick_base_target = None
#         self.stack_level = 0 if bool(goal.reset_stack) else self.stack_level

#         self.transition(TaskState.REQUEST_VISION, feedback, goal_handle)

#         while self.state not in (TaskState.DONE, TaskState.ERROR):
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 result.success = False
#                 result.message = "Task canceled"
#                 self.reset_fsm()
#                 return result

#             # 기존 구조 유지: spin_once로 콜백/타이머/액션 future 진행
#             rclpy.spin_once(self, timeout_sec=0.05)

#         if self.state == TaskState.DONE:
#             goal_handle.succeed()
#             result.success = True
#             result.message = "Task completed"
#         else:
#             goal_handle.abort()
#             result.success = False
#             result.message = "Task failed"

#         self.reset_fsm()
#         return result

#     # ==================================================
#     # FSM Transition
#     # ==================================================
#     def transition(self, next_state, feedback, goal_handle):
#         self.state = next_state

#         feedback.state = self.state.name
#         feedback.current_index = int(self.current_index)
#         feedback.total = int(len(self.marker_ids))
#         goal_handle.publish_feedback(feedback)

#         if self.state == TaskState.REQUEST_VISION:
#             self.request_vision(goal_handle, feedback)
#         elif self.state == TaskState.REQUEST_PNP:
#             self.request_pnp(goal_handle, feedback)
#         elif self.state == TaskState.UPDATE_STACK:
#             self.stack_level += 1
#             self.current_index += 1
#             self.transition(TaskState.REQUEST_VISION, feedback, goal_handle)
#         elif self.state == TaskState.DONE:
#             self.get_logger().info("✅ Task finished")
#         elif self.state == TaskState.ERROR:
#             self.get_logger().error("❌ Task error")

#     # ==================================================
#     # Vision Action
#     # ==================================================
#     def request_vision(self, goal_handle, feedback):
#         if self.current_index >= len(self.marker_ids):
#             self.transition(TaskState.DONE, feedback, goal_handle)
#             return

#         if not self.vision_client.wait_for_server(timeout_sec=1.0):
#             self.transition(TaskState.ERROR, feedback, goal_handle)
#             return

#         goal_msg = ObserveMarker.Goal()
#         goal_msg.marker_id = int(self.marker_ids[self.current_index])
#         goal_msg.n_samples = int(self.obs_samples)
#         goal_msg.timeout_sec = float(self.obs_timeout)
#         goal_msg.outlier_thresh = float(self.obs_outlier)

#         self.state = TaskState.WAIT_VISION
#         future = self.vision_client.send_goal_async(goal_msg)
#         future.add_done_callback(
#             lambda f: self.vision_goal_response_cb(f, goal_handle, feedback)
#         )

#     def vision_goal_response_cb(self, future, goal_handle, feedback):
#         gh = future.result()
#         if not gh.accepted:
#             self.transition(TaskState.ERROR, feedback, goal_handle)
#             return
#         gh.get_result_async().add_done_callback(
#             lambda f: self.vision_result_cb(f, goal_handle, feedback)
#         )

#     def vision_result_cb(self, future, goal_handle, feedback):
#         result = future.result().result
#         if not result.ok:
#             self.transition(TaskState.ERROR, feedback, goal_handle)
#             return
#         self.pick_base_target = list(result.base_target_coords)
#         self.transition(TaskState.REQUEST_PNP, feedback, goal_handle)

#     # ==================================================
#     # ExecutePnP Action
#     # ==================================================
#     def request_pnp(self, goal_handle, feedback):
#         if not self.pnp_client.wait_for_server(timeout_sec=1.0):
#             self.transition(TaskState.ERROR, feedback, goal_handle)
#             return

#         pick_pregrasp = self.make_pregrasp(self.pick_base_target)
#         place = self.make_place_pose()   # 🔥 핵심 변경

#         goal_msg = ExecutePnP.Goal()
#         goal_msg.pick_coords = list(pick_pregrasp)
#         goal_msg.place_coords = list(place)

#         self.state = TaskState.WAIT_PNP
#         future = self.pnp_client.send_goal_async(goal_msg)
#         future.add_done_callback(
#             lambda f: self.pnp_goal_response_cb(f, goal_handle, feedback)
#         )

#     def pnp_goal_response_cb(self, future, goal_handle, feedback):
#         gh = future.result()
#         if not gh.accepted:
#             self.transition(TaskState.ERROR, feedback, goal_handle)
#             return
#         gh.get_result_async().add_done_callback(
#             lambda f: self.pnp_result_cb(f, goal_handle, feedback)
#         )

#     def pnp_result_cb(self, future, goal_handle, feedback):
#         result = future.result().result
#         if not result.success:
#             self.transition(TaskState.ERROR, feedback, goal_handle)
#             return
#         self.transition(TaskState.UPDATE_STACK, feedback, goal_handle)

#     # ==================================================
#     # Strategy
#     # ==================================================
#     def make_pregrasp(self, base_target):
#         x, y, z, rx, ry, rz = base_target

#         T_base_target = np.eye(4)
#         T_base_target[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
#         T_base_target[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3

#         T_target_pregrasp = np.eye(4)
#         T_target_pregrasp[:3, 3] = [0.0, 0.0, 0.105]
#         T_target_pregrasp[:3, :3] = R.from_euler('x', 185, degrees=True).as_matrix()

#         T_ee_z_45 = np.eye(4)
#         T_ee_z_45[:3, :3] = R.from_euler('z', 45, degrees=True).as_matrix()

#         T_base_pregrasp = T_base_target @ T_target_pregrasp @ T_ee_z_45
#         return T_to_coords(T_base_pregrasp)

#     def make_place_pose(self):
#         """
#         assembly frame 기준 stacking
#         (원본 코드와 동일한 의미)
#         """
#         T_base_assembly = coords_to_T(self.assembly_pose)

#         dz_m = (
#             self.stack_level
#             * self.stack_dz_mm
#             * self.stack_dz_sign
#             * 1e-3
#         )

#         T_stack = np.eye(4)
#         T_stack[:3, 3] = [0.0, 0.0, dz_m]

#         T_base_place = T_base_assembly @ T_stack
#         return T_to_coords(T_base_place)


# def main(args=None):
#     rclpy.init(args=args)
#     node = TaskManagerNode()
#     try:
#         rclpy.spin(node)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



## task_manager_node.py
import asyncio
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from enum import Enum, auto
from scipy.spatial.transform import Rotation as R

from mycobot_system_interfaces.action import ObserveMarker, ExecutePnP, ExecuteTask
from std_msgs.msg import Bool


# ==================================================
# Utility
# ==================================================
def coords_to_T(coords_mm_deg):
    x, y, z, rx, ry, rz = coords_mm_deg
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
    T[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3
    return T


def T_to_coords(T):
    t_mm = T[:3, 3] * 1e3
    rpy = R.from_matrix(T[:3, :3]).as_euler('xyz', degrees=True)
    return [
        float(t_mm[0]), float(t_mm[1]), float(t_mm[2]),
        float(rpy[0]), float(rpy[1]), float(rpy[2]),
    ]


# ==================================================
# FSM State
# ==================================================
class TaskState(Enum):
    IDLE = auto()
    REQUEST_VISION = auto()
    WAIT_VISION = auto()
    REQUEST_PNP = auto()
    WAIT_PNP = auto()
    UPDATE_STACK = auto()
    DONE = auto()
    ERROR = auto()


class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager_node')

        # =====================================
        # Assembly state publisher (항상 스트리밍)
        # =====================================
        self.assembly_pub = self.create_publisher(Bool, '/assembly_start', 10)
        self._running = False

        # 10 Hz 상태 스트림
        self.assembly_timer = self.create_timer(0.1, self.publish_assembly_state)

        # Action clients
        self.vision_client = ActionClient(self, ObserveMarker, '/vision/observe_marker')
        self.pnp_client = ActionClient(self, ExecutePnP, '/manipulator/execute_pnp_action')

        # FSM state
        self.state = TaskState.IDLE
        self.marker_ids = []
        self.current_index = 0
        self.stack_level = 0
        self.pick_base_target = None

        # Assembly + stacking
        self.assembly_pose = [60, 180, 110, -180, 0, -45]
        self.stack_dz_mm = 25.0
        self.stack_dz_sign = -1.0

        # Observe parameters
        self.obs_samples = 12
        self.obs_timeout = 0.5
        self.obs_outlier = 0.03

        # 재진입 방지
        self._busy_goal = False

        # Action server
        self.action_server = ActionServer(
            self,
            ExecuteTask,
            '/task/execute',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

        self.get_logger().info("🧠 TaskManager ActionServer ready: /task/execute")
        self.get_logger().info("📣 Streaming /assembly_start at 10 Hz (True=running, False=idle)")

    # ==================================================
    # Real-time publisher
    # ==================================================
    def publish_assembly_state(self):
        msg = Bool()
        msg.data = bool(self._running)
        self.assembly_pub.publish(msg)

    # ==================================================
    # Reset helpers
    # ==================================================
    def reset_fsm(self):
        self.state = TaskState.IDLE
        self.marker_ids = []
        self.current_index = 0
        self.pick_base_target = None
        self._running = False
        self._busy_goal = False
        self.get_logger().info("🔄 FSM reset to IDLE")

    # ==================================================
    # Action callbacks
    # ==================================================
    def goal_cb(self, goal_request):
        if self._busy_goal:
            self.get_logger().warn("⚠️ Task already running -> reject new goal")
            return GoalResponse.REJECT
        if len(goal_request.marker_ids) == 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        self.get_logger().warn("🛑 Task cancel requested")
        return CancelResponse.ACCEPT

    # ==================================================
    # Async helpers (핵심: spin_once 제거, await 기반)
    # ==================================================
    async def wait_action_server(self, client: ActionClient, timeout_sec: float, name: str) -> bool:
        """spin을 main에서 계속 돌리므로, 여기선 짧게 폴링하며 await로 양보"""
        t0 = self.get_clock().now().nanoseconds * 1e-9
        while not client.wait_for_server(timeout_sec=0.1):
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - t0) > timeout_sec:
                self.get_logger().error(f"❌ {name} action server not available (timeout {timeout_sec}s)")
                return False
            await asyncio.sleep(0.0)
        return True

    async def call_vision(self, marker_id: int):
        ok = await self.wait_action_server(self.vision_client, 3.0, "Vision(/vision/observe_marker)")
        if not ok:
            return None  # failure

        goal_msg = ObserveMarker.Goal()
        goal_msg.marker_id = int(marker_id)
        goal_msg.n_samples = int(self.obs_samples)
        goal_msg.timeout_sec = float(self.obs_timeout)
        goal_msg.outlier_thresh = float(self.obs_outlier)

        send_fut = self.vision_client.send_goal_async(goal_msg)
        goal_handle = await send_fut
        if not goal_handle.accepted:
            self.get_logger().error("❌ Vision goal rejected")
            return None

        res_fut = goal_handle.get_result_async()
        res = await res_fut
        result = res.result
        if not result.ok:
            self.get_logger().error("❌ Vision returned ok:false")
            return None

        return list(result.base_target_coords)

    async def call_pnp(self, pick_coords, place_coords) -> bool:
        ok = await self.wait_action_server(self.pnp_client, 3.0, "PnP(/manipulator/execute_pnp_action)")
        if not ok:
            return False

        goal_msg = ExecutePnP.Goal()
        goal_msg.pick_coords = list(pick_coords)
        goal_msg.place_coords = list(place_coords)

        send_fut = self.pnp_client.send_goal_async(goal_msg)
        goal_handle = await send_fut
        if not goal_handle.accepted:
            self.get_logger().error("❌ PnP goal rejected")
            return False

        res_fut = goal_handle.get_result_async()
        res = await res_fut
        result = res.result
        if not result.success:
            self.get_logger().error("❌ PnP returned success:false")
            return False

        return True

    # ==================================================
    # ExecuteTask Action (async, but NO spin_once)
    # ==================================================
    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        feedback = ExecuteTask.Feedback()
        result = ExecuteTask.Result()

        self._busy_goal = True
        self._running = True

        self.marker_ids = list(goal.marker_ids)
        self.current_index = 0
        self.pick_base_target = None
        self.stack_level = 0 if bool(goal.reset_stack) else self.stack_level

        try:
            # marker loop
            while self.current_index < len(self.marker_ids):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Task canceled"
                    self.reset_fsm()
                    return result

                # ----- Vision -----
                self.state = TaskState.REQUEST_VISION
                feedback.state = self.state.name
                feedback.current_index = int(self.current_index)
                feedback.total = int(len(self.marker_ids))
                goal_handle.publish_feedback(feedback)

                self.state = TaskState.WAIT_VISION
                feedback.state = self.state.name
                goal_handle.publish_feedback(feedback)

                marker_id = int(self.marker_ids[self.current_index])
                base_target = await self.call_vision(marker_id)
                if base_target is None:
                    self.state = TaskState.ERROR
                    goal_handle.abort()
                    result.success = False
                    result.message = "Vision failed"
                    self.reset_fsm()
                    return result

                self.pick_base_target = base_target

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Task canceled"
                    self.reset_fsm()
                    return result

                # ----- PnP -----
                self.state = TaskState.REQUEST_PNP
                feedback.state = self.state.name
                goal_handle.publish_feedback(feedback)

                pick_pregrasp = self.make_pregrasp(self.pick_base_target)
                place = self.make_place_pose()

                self.state = TaskState.WAIT_PNP
                feedback.state = self.state.name
                goal_handle.publish_feedback(feedback)

                ok = await self.call_pnp(pick_pregrasp, place)
                if not ok:
                    self.state = TaskState.ERROR
                    goal_handle.abort()
                    result.success = False
                    result.message = "PnP failed"
                    self.reset_fsm()
                    return result

                # ----- Update stack -----
                self.state = TaskState.UPDATE_STACK
                feedback.state = self.state.name
                goal_handle.publish_feedback(feedback)

                self.stack_level += 1
                self.current_index += 1

            # done
            self.state = TaskState.DONE
            feedback.state = self.state.name
            goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.success = True
            result.message = "Task completed"
            self.reset_fsm()
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Exception in execute_cb: {e}")
            self.state = TaskState.ERROR
            goal_handle.abort()
            result.success = False
            result.message = f"Exception: {e}"
            self.reset_fsm()
            return result

    # ==================================================
    # Strategy
    # ==================================================
    def make_pregrasp(self, base_target):
        x, y, z, rx, ry, rz = base_target

        T_base_target = np.eye(4)
        T_base_target[:3, :3] = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()
        T_base_target[:3, 3] = np.array([x, y, z], dtype=float) * 1e-3

        T_target_pregrasp = np.eye(4)
        T_target_pregrasp[:3, 3] = [0.0, 0.0, 0.105]
        T_target_pregrasp[:3, :3] = R.from_euler('x', 185, degrees=True).as_matrix()

        T_ee_z_45 = np.eye(4)
        T_ee_z_45[:3, :3] = R.from_euler('z', 45, degrees=True).as_matrix()

        T_base_pregrasp = T_base_target @ T_target_pregrasp @ T_ee_z_45
        return T_to_coords(T_base_pregrasp)

    def make_place_pose(self):
        T_base_assembly = coords_to_T(self.assembly_pose)

        dz_m = (
            self.stack_level
            * self.stack_dz_mm
            * self.stack_dz_sign
            * 1e-3
        )

        T_stack = np.eye(4)
        T_stack[:3, 3] = [0.0, 0.0, dz_m]

        T_base_place = T_base_assembly @ T_stack
        return T_to_coords(T_base_place)


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        # ✅ 항상 spin: 타이머(assembly_start) 스트림이 절대 끊기지 않음
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
