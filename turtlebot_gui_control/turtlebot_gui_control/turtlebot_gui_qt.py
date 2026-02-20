import os
import sys
import time

import rclpy
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QLabel,
    QDialog, QVBoxLayout, QDialogButtonBox
)
from PySide6.QtCore import Qt
from PySide6.QtCore import QThread, Signal, Slot, QTimer
from PySide6.QtGui import QPixmap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .turtlebot_gui_ui import Ui_MainWindow
from .auto_control import AutoControl as AutoControl
from .manual_control import ManualControl as ManualControl
from .detect_obstacle import DetectObstacle as DetectObstacle
from turtlebot_gui_interfaces.msg import ObstacleDistance, GoalPose
from turtlebot_gui_interfaces.srv import StartStop

EXPECTED_LOGIN_ID = "admin"
EXPECTED_LOGIN_PW = "1234"

class RclpyThread(QThread):
    def __init__(self, executor):
        super().__init__()
        self.executor = executor

    def run(self):
        try:
            self.executor.spin()
        finally:
            rclpy.shutdown()


class DistanceSubscriber(Node):
    def __init__(self, on_distance):
        super().__init__('distance_subscriber')
        self._on_distance = on_distance
        self.create_subscription(
            ObstacleDistance,
            '/obstacle_distance',
            self._callback,
            10
        )

    def _callback(self, msg):
        self._on_distance(msg.distance)


class PoseSubscriber(Node):
    def __init__(self, on_pose):
        super().__init__('pose_subscriber')
        self._on_pose = on_pose
        self.create_subscription(
            Odometry,
            '/odom',
            self._callback,
            10
        )

    def _callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._on_pose(x, y)


class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client')
        self.goal_pub = self.create_publisher(GoalPose, '/goal_pose', 10)
        self.start_stop_client = self.create_client(StartStop, '/start_stop')

    def publish_goal(self, x, y):
        msg = GoalPose()
        msg.x = float(x)
        msg.y = float(y)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Goal published: ({x:.2f}, {y:.2f})')

    def request_start_stop(self, start):
        if not self.start_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('StartStop service NOT available after 1.0s timeout!')
            return False
        self.get_logger().info(f'StartStop service available, sending request: start={start}')
        req = StartStop.Request()
        req.start = bool(start)
        future = self.start_stop_client.call_async(req)
        return True


class CmdVelSubscriber(Node):
    def __init__(self, on_cmd_vel):
        super().__init__('cmd_vel_subscriber')
        self._on_cmd_vel = on_cmd_vel
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._callback,
            10
        )

    def _callback(self, msg):
        self._on_cmd_vel(msg.linear.x, msg.angular.z)


class NavStatusSubscriber(Node):
    def __init__(self, on_status):
        super().__init__('nav_status_subscriber')
        self._on_status = on_status
        self.create_subscription(
            String,
            '/nav_status',
            self._callback,
            10
        )

    def _callback(self, msg):
        self._on_status(msg.data)


class NavLogSubscriber(Node):
    def __init__(self, on_log):
        super().__init__('nav_log_subscriber')
        self._on_log = on_log
        self.create_subscription(
            String,
            '/nav_log',
            self._callback,
            10
        )

    def _callback(self, msg):
        self._on_log(msg.data)


class MissionEventSubscriber(Node):
    def __init__(self, on_event):
        super().__init__('mission_event_subscriber')
        self._on_event = on_event
        self.create_subscription(
            String,
            '/mission_event',
            self._callback,
            10
        )

    def _callback(self, msg):
        self._on_event(msg.data)


class MainWindow(QMainWindow):
    distance_updated = Signal(float)
    pose_updated = Signal(float, float)
    cmd_vel_updated = Signal(float, float)
    nav_status_updated = Signal(str)
    nav_log_updated = Signal(str)
    mission_event_updated = Signal(str)

    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        # setupUi로 MainWindow 위젯 배치
        self.ui.setupUi(self)
        self.ui.stackedWidget.setCurrentIndex(0)
        self._is_logged_in = False
        self._success_handled = False
        self._completion_shown = False  # 중복 완료 팝업 방지
        # 버튼 홀드 시 속도 지속 증가
        self.ui.btn_login.clicked.connect(self.btn_login_clicked)
        self.ui.btn_forward.pressed.connect(self._start_forward_hold)
        self.ui.btn_forward.released.connect(self._stop_forward_hold)
        self.ui.btn_backward.pressed.connect(self._start_backward_hold)
        self.ui.btn_backward.released.connect(self._stop_backward_hold)
        self.ui.btn_left.pressed.connect(self._start_left_hold)
        self.ui.btn_left.released.connect(self._stop_left_hold)
        self.ui.btn_right.pressed.connect(self._start_right_hold)
        self.ui.btn_right.released.connect(self._stop_right_hold)
        self.ui.btn_stop.clicked.connect(self.btn_stop_clicked)
        self.ui.btn_menu_toggle.clicked.connect(self.btn_menu_toggle_clicked)
        self.ui.btn_nav_start.clicked.connect(self.btn_nav_start_clicked)
        self.ui.btn_nav_stop.clicked.connect(self.btn_nav_stop_clicked)
        self.ui.btn_log_clear.clicked.connect(self.btn_log_clear_clicked)
        self.ui.btn_exit_1.clicked.connect(self.btn_exit_clicked)
        self.ui.btn_exit_2.clicked.connect(self.btn_exit_clicked)
        self.ui.distance_label.setText("Dis: 0.00 m")
        self.ui.x_label.setText("X: 0.00")
        self.ui.y_label.setText("Y: 0.00")
        self._update_goal_labels(None, None)
        self.log_list = self.ui.log_list

        self.distance_updated.connect(self._update_distance_label)
        self.pose_updated.connect(self._update_pose_labels)
        self.cmd_vel_updated.connect(self._update_cmd_vel_labels)
        self.nav_status_updated.connect(self._handle_nav_status)
        self.nav_log_updated.connect(self._handle_nav_log)
        self.mission_event_updated.connect(self._handle_mission_event)

        self._hold_interval_ms = 100
        self._forward_timer = QTimer(self)
        self._forward_timer.setInterval(self._hold_interval_ms)
        self._forward_timer.timeout.connect(self.btn_forward_clicked)
        self._backward_timer = QTimer(self)
        self._backward_timer.setInterval(self._hold_interval_ms)
        self._backward_timer.timeout.connect(self.btn_backward_clicked)
        self._left_timer = QTimer(self)
        self._left_timer.setInterval(self._hold_interval_ms)
        self._left_timer.timeout.connect(self.btn_left_clicked)
        self._right_timer = QTimer(self)
        self._right_timer.setInterval(self._hold_interval_ms)
        self._right_timer.timeout.connect(self.btn_right_clicked)

        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.auto = AutoControl()  # 자동 주행 노드 초기화
        self.manual = ManualControl()  # 수동 주행 노드 초기화
        self.detect_obstacle = DetectObstacle()  # 장애물 감지 노드 초기화
        self.distance_sub = DistanceSubscriber(self.distance_updated.emit)
        self.pose_sub = PoseSubscriber(self.pose_updated.emit)
        self.mission_client = MissionClient()
        self.cmd_vel_sub = CmdVelSubscriber(self.cmd_vel_updated.emit)
        self.nav_status_sub = NavStatusSubscriber(self.nav_status_updated.emit)
        self.nav_log_sub = NavLogSubscriber(self.nav_log_updated.emit)
        self.mission_event_sub = MissionEventSubscriber(self.mission_event_updated.emit)
        self._manual_added = False
        self._auto_added = False
        self.ui.btn_menu_toggle.setText("AUTO")
        self.ui.mode_label.setText("MODE: MANUAL")
        self.executor.add_node(self.detect_obstacle)
        self.executor.add_node(self.distance_sub)
        self.executor.add_node(self.pose_sub)
        self.executor.add_node(self.mission_client)
        self.executor.add_node(self.cmd_vel_sub)
        self.executor.add_node(self.nav_status_sub)
        self.executor.add_node(self.nav_log_sub)
        self.executor.add_node(self.mission_event_sub)
        self.rclpy_thread = RclpyThread(self.executor)
        self.rclpy_thread.start()
        self._set_controls_enabled(False)

    def btn_forward_clicked(self):
        self.manual.move_forward()

    def btn_login_clicked(self):
        user_id = self.ui.id_input_line.text().strip()
        user_pw = self.ui.pw_input_line.text().strip()
        if user_id == EXPECTED_LOGIN_ID and user_pw == EXPECTED_LOGIN_PW:
            self._is_logged_in = True
            self.ui.stackedWidget.setCurrentIndex(1)
            self._set_controls_enabled(True)
            self._ensure_manual_node()
            self._clear_login_inputs()
            return
        self.ui.pw_input_line.setText("")

    def btn_exit_clicked(self):
        current_index = self.ui.stackedWidget.currentIndex()
        prev_index = max(0, current_index - 1)
        self.ui.stackedWidget.setCurrentIndex(prev_index)
        if prev_index == 0:
            self._is_logged_in = False
            self._set_controls_enabled(False)
            self._remove_manual_node()
            self._clear_login_inputs()

    def btn_backward_clicked(self):
        self.manual.move_backward()

    def btn_left_clicked(self):
        self.manual.turn_left()

    def btn_right_clicked(self):
        self.manual.turn_right()

    def btn_stop_clicked(self):
        self._publish_zero_cmd()
        self._stop_hold_timers()

        if self.ui.btn_menu_toggle.text() == "MANUAL":
            # AUTO \ubaa8\ub4dc\uc5d0\uc11c STOP \ub204\ub974\uba74 MANUAL\ub85c \uc804\ud658
            if self.auto:
                self.auto.stop()
            self.ui.btn_menu_toggle.setText("AUTO")
            self.ui.mode_label.setText("MODE: MANUAL")
            if self._auto_added and self.auto:
                self.executor.remove_node(self.auto)
                self._auto_added = False
            self._ensure_manual_node()
            self._log("[STOP] Switched to MANUAL mode")
        else:
            self.manual.stop()


    def btn_menu_toggle_clicked(self):
        self._stop_hold_timers()
        self._publish_zero_cmd()
        if self.ui.btn_menu_toggle.text() == "MANUAL":
            self.ui.btn_menu_toggle.setText("AUTO")
            self.ui.mode_label.setText("MODE: MANUAL")
            # 수동 주행 모드로 전환
            if self._auto_added and self.auto:
                self.executor.remove_node(self.auto)
                self._auto_added = False
                self._log("[MODE] AutoControl removed from executor")
            self._ensure_manual_node()
            self._log("[MODE] Switched to MANUAL")
        else:
            self.ui.btn_menu_toggle.setText("MANUAL")
            self.ui.mode_label.setText("MODE: AUTO")
            # 자동 주행 모드로 전환
            if self._manual_added and self.manual:
                self.executor.remove_node(self.manual)
                self._manual_added = False
                self._log("[MODE] ManualControl removed from executor")
            # AutoControl 재사용 (재생성 안 함)
            if not self._auto_added and self.auto:
                self.executor.add_node(self.auto)
                self._auto_added = True
                self._log("[MODE] AutoControl added to executor")
            self._log("[MODE] Switched to AUTO")

    def btn_nav_start_clicked(self):
        x, y = self._get_goal_inputs()
        self._update_goal_labels(x, y)
        self._success_handled = False
        if x is None or y is None:
            self._log("Goal not set: X and Y are required.")
            self._clear_goal_inputs()
            return
        self._log(f"[NAV START] Goal: x={x:.2f}, y={y:.2f}, AutoControl in executor: {self._auto_added}")
        self.mission_client.publish_goal(x, y)
        success = self.mission_client.request_start_stop(True)
        if success:
            self._log("[NAV START] StartStop request sent successfully")
        else:
            self._log("[NAV START] ERROR: StartStop service unavailable!")
        self._clear_goal_inputs()

    def btn_nav_stop_clicked(self):
        self.mission_client.request_start_stop(False)
        self._log("Mission stop requested.")

    def btn_log_clear_clicked(self):
        self.log_list.clear()

    def _get_goal_inputs(self):
        x_text = self.ui.x_line.text().strip()
        y_text = self.ui.y_line.text().strip()
        x = None
        y = None
        if x_text:
            try:
                x = float(x_text)
            except ValueError:
                x = None
        if y_text:
            try:
                y = float(y_text)
            except ValueError:
                y = None
        return x, y

    def _log(self, message):
        self.log_list.addItem(message)
        self.log_list.scrollToBottom()

    def _log_with_time(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self._log(f"[{timestamp}] {message}")

    def _update_goal_labels(self, x, y):
        x_text = "None" if x is None else f"{x:.1f}"
        y_text = "None" if y is None else f"{y:.1f}"
        self.ui.goal_x_label.setText(f"Goal X: {x_text}")
        self.ui.goal_y_label.setText(f"Goal Y: {y_text}")

    def _clear_goal_inputs(self):
        self.ui.x_line.setText("")
        self.ui.y_line.setText("")

    def _clear_login_inputs(self):
        self.ui.id_input_line.setText("")
        self.ui.pw_input_line.setText("")

    def _ensure_manual_node(self):
        if self._manual_added:
            return
        if self.auto and self._auto_added:
            self.executor.remove_node(self.auto)
            self._auto_added = False
        self.executor.add_node(self.manual)
        self._manual_added = True

    def _remove_manual_node(self):
        if not self._manual_added:
            return
        self.executor.remove_node(self.manual)
        self._manual_added = False

    def _set_controls_enabled(self, enabled):
        self.ui.btn_forward.setEnabled(enabled)
        self.ui.btn_backward.setEnabled(enabled)
        self.ui.btn_left.setEnabled(enabled)
        self.ui.btn_right.setEnabled(enabled)
        self.ui.btn_stop.setEnabled(enabled)
        self.ui.btn_menu_toggle.setEnabled(enabled)
        self.ui.btn_nav_start.setEnabled(enabled)
        self.ui.btn_nav_stop.setEnabled(enabled)
        self.ui.btn_log_clear.setEnabled(enabled)

    def _publish_zero_cmd(self):
        if self._auto_added and self.auto:
            self.auto._publish_cmd(0.0, 0.0)
            return
        if self._manual_added and self.manual:
            self.manual.stop()
            self.manual._publish_cmd()

    def _switch_to_manual_mode(self):
        if self._auto_added and self.auto:
            self.executor.remove_node(self.auto)
            self._auto_added = False
        self._ensure_manual_node()
        self.ui.btn_menu_toggle.setText("AUTO")
        self.ui.mode_label.setText("MODE: MANUAL")

    def _handle_nav_status(self, status):
        # nav_status SUCCESS: only log and stop robot, no popup
        # Completion popup is handled by mission_event only
        if status == "SUCCESS" and not self._success_handled:
            self._success_handled = True
            self._log_with_time("SUCCESS (goal reached)")
            self._publish_zero_cmd()
            self._switch_to_manual_mode()
            self.mission_client.request_start_stop(False)

    def _handle_nav_log(self, log_msg):
        """네비게이션 로그를 GUI에 표시"""
        self._log_with_time(log_msg)

    def _handle_mission_event(self, event_json):
        """미션 완료 이벤트 처리: wall map 이미지 팝업 (SINGLE SOURCE OF TRUTH)"""
        # Guard: prevent duplicate popups
        if self._completion_shown:
            print("[GUI] Completion already shown, ignoring duplicate event")
            return

        try:
            import json
            print(f"[GUI] 📨 Mission event received (raw): {event_json}")
            event = json.loads(event_json)

            if event.get("event") != "mission_done":
                print(f"[GUI] Not a mission_done event, ignoring")
                return

            # Set flag to prevent duplicate popups
            self._completion_shown = True

            map_path = event.get("map_path", "")
            goal_reached = event.get("goal_reached", False)

            # ~ 경로 확장 (중요!)
            import os
            map_path = os.path.expanduser(map_path)
            print(f"[GUI] Expanded map path: {map_path}")

            print(f"[GUI] ✅ Mission event parsed: map={map_path}, goal_reached={goal_reached}")
            self._log_with_time(f"Mission completed! Map: {map_path}")

            # 파일 존재 확인
            if os.path.exists(map_path):
                file_size = os.path.getsize(map_path)
                print(f"[GUI] Map file exists: {map_path} ({file_size} bytes)")
            else:
                print(f"[GUI] Map file NOT found: {map_path}")

            # Custom QDialog 팝업 (no text clipping guaranteed)
            print("[GUI] Showing mission complete dialog...")
            dialog = QDialog(self)
            dialog.setWindowTitle("미션 완료")
            layout = QVBoxLayout(dialog)

            label = QLabel("완료되었습니다.\n지도를 확인하시겠습니까?")
            label.setWordWrap(True)
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label)

            buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
            buttons.accepted.connect(dialog.accept)
            buttons.rejected.connect(dialog.reject)
            layout.addWidget(buttons)

            dialog.setMinimumWidth(350)

            result = dialog.exec()
            print(f"[GUI] Dialog result: {'OK' if result == QDialog.Accepted else 'Cancel'}")

            if result == QDialog.Accepted:
                self._show_map_on_page(map_path)
        except Exception as e:
            import traceback
            print(f"[GUI] Mission event error: {e}")
            print(f"[GUI] Traceback: {traceback.format_exc()}")

    def _show_map_on_page(self, filepath):
        """세 번째 페이지(stackedWidgetPage2)의 map_label에 지도 이미지 표시"""
        import os

        # ~ 경로 확장 (중요!)
        filepath = os.path.expanduser(filepath)
        print(f"[GUI] _show_map_on_page called with: {filepath}")
        print(f"Expanded map path: {filepath}")

        if not os.path.exists(filepath):
            print(f"[GUI] ❌ Map file not found: {filepath}")
            QMessageBox.warning(self, "Error", f"Map file not found: {filepath}")
            return

        file_size = os.path.getsize(filepath)
        print(f"[GUI] Map file exists: {filepath} ({file_size} bytes)")

        # QPixmap 로드
        print(f"[GUI] Loading pixmap from {filepath}...")
        pixmap = QPixmap(filepath)
        if pixmap.isNull():
            print(f"[GUI] ❌ Failed to load pixmap: {filepath}")
            QMessageBox.warning(self, "Error", f"Failed to load image: {filepath}")
            return

        print(f"[GUI] ✅ Pixmap loaded successfully: {pixmap.width()}x{pixmap.height()}")

        # map_label의 크기에 맞게 스케일링 (671x581)
        scaled_pixmap = pixmap.scaled(671, 581, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        print(f"[GUI] Scaled to: {scaled_pixmap.width()}x{scaled_pixmap.height()}")

        # map_label에 이미지 설정
        self.ui.map_label.setPixmap(scaled_pixmap)
        self.ui.map_label.setAlignment(Qt.AlignCenter)

        # 로봇 정지 및 모드 전환 (로그인 페이지로 가는 것과 동일)
        print("[GUI] Stopping robot and switching to manual mode...")
        self._publish_zero_cmd()
        self._switch_to_manual_mode()
        self.mission_client.request_start_stop(False)

        # 세 번째 페이지로 강제 전환 (index 2 = stackedWidgetPage2 = map page)
        print(f"[GUI] Current page index before switch: {self.ui.stackedWidget.currentIndex()}")
        self.ui.stackedWidget.setCurrentIndex(2)
        print(f"[GUI] ✅ Switched to map page (index {self.ui.stackedWidget.currentIndex()})")
        self._log_with_time("🗺️ Map displayed on page 3")

    def _handle_success(self):
        # DEPRECATED: Completion popup is now handled by mission_event only
        # This function is kept for compatibility but does nothing
        # Original custom QDialog popup has been removed to unify UX
        pass

    def _start_forward_hold(self):
        self.btn_forward_clicked()
        self._forward_timer.start()

    def _stop_forward_hold(self):
        self._forward_timer.stop()

    def _start_backward_hold(self):
        self.btn_backward_clicked()
        self._backward_timer.start()

    def _stop_backward_hold(self):
        self._backward_timer.stop()

    def _start_left_hold(self):
        self.btn_left_clicked()
        self._left_timer.start()

    def _stop_left_hold(self):
        self._left_timer.stop()

    def _start_right_hold(self):
        self.btn_right_clicked()
        self._right_timer.start()

    def _stop_right_hold(self):
        self._right_timer.stop()

    def _stop_hold_timers(self):
        self._forward_timer.stop()
        self._backward_timer.stop()
        self._left_timer.stop()
        self._right_timer.stop()

    @Slot(float)
    def _update_distance_label(self, distance):
        self.ui.distance_label.setText(f"Dis: {distance:.2f} m")

    @Slot(float, float)
    def _update_pose_labels(self, x, y):
        self.ui.x_label.setText(f"X: {x:.2f}")
        self.ui.y_label.setText(f"Y: {y:.2f}")

    @Slot(float, float)
    def _update_cmd_vel_labels(self, linear, angular):
        self.ui.velocity_label.setText(f"Vel: {linear:.2f}")
        self.ui.angular_labe.setText(f"Ang: {angular:.2f}")

    # _save_map() removed - no longer using nav2_map_server
    # Wall-based PNG map is generated by AutoControl's WallMapBuilder

    def _show_map(self, map_path):
        """Display map image on map_label with proper error handling"""
        import os

        # Always expand ~ in path
        map_path = os.path.expanduser(map_path)
        print(f"[GUI] _show_map loading: {map_path}")

        # Check if file exists
        if not os.path.exists(map_path):
            print(f"[GUI] Map file not found: {map_path}")
            self._log_with_time(f"Map file not found: {map_path}")
            return

        file_size = os.path.getsize(map_path)
        print(f"[GUI] Map file exists ({file_size} bytes)")

        # Load pixmap
        pixmap = QPixmap(map_path)
        if pixmap.isNull():
            print(f"[GUI] Failed to load pixmap from: {map_path}")
            self._log_with_time("Map load failed - pixmap is null")
            return

        print(f"[GUI] Pixmap loaded: {pixmap.width()}x{pixmap.height()}")
        self.ui.map_label.setPixmap(pixmap)
        self.ui.map_label.setScaledContents(True)
        self._log_with_time("🗺️ Map displayed")
def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
