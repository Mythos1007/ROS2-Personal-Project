import math
import time
import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from turtlebot_gui_interfaces.msg import GoalPose
from turtlebot_gui_interfaces.srv import StartStop
from .wall_map import WallMapBuilder

# ==================== 상태 정의 ====================
STATE_IDLE = "IDLE"
STATE_SNAP = "SNAP"        # 가까운 정수 교차점으로 스냅
STATE_SCAN = "SCAN"        # 라이다로 주변 인식
STATE_TURN = "TURN"        # 90도/180도 회전
STATE_DRIVE = "DRIVE"      # 다음 교차점까지 1m 전진
STATE_SUCCESS = "SUCCESS"

# ==================== 격자/좌표 상수 ====================
GRID_STEP = 1.0            # 격자 간격 (1m)
EVEN_SNAP_INTERVAL = 1.0   # 교차점 간격 (정수 좌표, 1m)

# ==================== 방향 정의 ====================
# 0=N(+y), 1=E(+x), 2=S(-y), 3=W(-x)
DIR_N = 0
DIR_E = 1
DIR_S = 2
DIR_W = 3

# ==================== 거리 판정 상수 ====================
INTERSECTION_OPEN_THRESH = 0.70   # 교차점 출구 판단용 최소 거리 (열림)
INTERSECTION_CLOSE_THRESH = 0.55  # 히스테리시스용 닫힘 임계값
STOP_MARGIN = 0.45                # 주행 중 긴급정지 거리 (1m 격자 기준)
LINE_TOLERANCE = 0.25             # 축 정렬 허용 오차 (rear-turn 판정용)

# ==================== 이동/회전 허용 오차 ====================
POSITION_TOLERANCE = 0.10    # 교차점 도달 판정 거리 (10cm)
YAW_TOLERANCE = 0.05         # 회전 완료 판정 각도 오차 (약 2.9도)
POS_ALIGN_TOLERANCE = 0.08   # SCAN 전 위치 정렬 허용 오차 (8cm)
YAW_ALIGN_TOLERANCE = 0.026  # SCAN 전 yaw 정렬 허용 오차 (약 1.5도)
PRESCAN_SETTLE_TIME = 0.2    # SCAN 전 정렬 후 안정화 시간 (초)

# ==================== 속도 상수 ====================
V_DRIVE = 0.22             # 직진 선속도
W_TURN = 0.50              # 회전 각속도

# ==================== 라이다 섹터 각도 (degrees, 정규화 후 -180~180) ====================
# LaserScan 0~360도를 내부에서 -180~180으로 정규화
FRONT_MIN_DEG = -20.0  # 전방: -20~20도
FRONT_MAX_DEG = 20.0
LEFT_MIN_DEG = 45.0    # 왼쪽: 45~135도 (확대)
LEFT_MAX_DEG = 135.0
RIGHT_MIN_DEG = -135.0 # 오른쪽: -135~-45도 (확대)
RIGHT_MAX_DEG = -45.0
BACK1_MIN_DEG = 150.0  # 후방 구간1: 150~180도
BACK1_MAX_DEG = 180.0
BACK2_MIN_DEG = -180.0 # 후방 구간2: -180~-150도 (래핑 고려)
BACK2_MAX_DEG = -150.0

# 섹터 거리 계산 방식
SECTOR_PERCENTILE = 50     # 퍼센타일 (50% = median)
SCAN_STABILIZE_TIME = 0.2  # SCAN 상태 진입 후 안정화 시간 (초)
SCAN_SAMPLE_COUNT = 5      # 안정화용 샘플 개수


class AutoControl(Node):
    """
    격자 기반 TurtleBot 자율주행 컨트롤러
    - 교차점(정수 좌표)에서만 판단
    - 회전은 90도 단위
    - 전진은 1m 스텝
    """
    def __init__(self):
        super().__init__('auto_control')
        self.qos_profile = QoSProfile(depth=10)

        # 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', self.qos_profile)
        self.nav_status_pub = self.create_publisher(String, '/nav_status', self.qos_profile)
        self.log_pub = self.create_publisher(String, '/nav_log', self.qos_profile)
        self.mission_event_pub = self.create_publisher(String, '/mission_event', self.qos_profile)

        # 서비스
        self.start_stop_service = self.create_service(
            StartStop, '/start_stop', self._handle_start_stop
        )

        # 구독자
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, self.qos_profile
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, self.qos_profile
        )
        self.goal_sub = self.create_subscription(
            GoalPose, '/goal_pose', self._goal_callback, self.qos_profile
        )

        # 타이머 (20Hz = 0.05초)
        self.timer = self.create_timer(0.05, self._on_timer)

        # 상태 변수
        self._start = False
        self._state = STATE_IDLE
        self._state_start_time = time.monotonic()

        # 센서 데이터
        self._odom_pose = None  # (x, y)
        self._yaw = 0.0
        self._front_dist = math.inf
        self._left_dist = math.inf
        self._right_dist = math.inf
        self._back_dist = math.inf
        self._front_valid = 0  # 전방 섹터 유효 데이터 개수
        self._left_valid = 0   # 왼쪽 섹터 유효 데이터 개수
        self._right_valid = 0  # 오른쪽 섹터 유효 데이터 개수
        self._back_valid = 0   # 후방 섹터 유효 데이터 개수
        self._sector_indices = None

        # SCAN 상태 샘플링용
        self._scan_samples = []  # (front, left, right, back) 튜플 리스트
        self._scan_stabilized = False

        # Pre-scan alignment 상태
        self._prescan_pos_aligned = False
        self._prescan_yaw_aligned = False
        self._prescan_settle_start = None

        # 목표 관련
        self._goal_pose = None  # 최종 목표 (x, y)
        self._success_published = False
        self._goal_reached_pending = False  # 목표 도달 후 스캔 대기 플래그

        # SNAP 상태용
        self._snap_target = None  # 스냅할 교차점 좌표 (gx, gy)

        # TURN 상태용
        self._target_yaw = None  # 목표 yaw

        # DRIVE 상태용
        self._drive_target = None  # 다음 교차점 좌표 (gx, gy)

        # DFS 탐색 상태 (set 기반)
        self._discovered = {}         # dict[node] = set(dir) - 열린 출구들
        self._tried = {}              # dict[node] = set(dir) - 이미 나가본 출구들
        self._parent = {}             # dict[node] = parent node
        self._parent_dir = {}         # dict[node] = 부모 방향 (backtrack용)
        self._current_node = None     # 현재 교차점 (ix, iy) 정수 인덱스
        self._current_dir = None      # 현재 방향 (0:N, 1:E, 2:S, 3:W)
        self._chosen_dir = None       # 선택한 이동 방향
        self._start_node = None       # 시작 노드
        self._backtrack_target = None # 되돌아갈 미해결 분기점
        self._exploration_done = False # 탐색 완료 여부
        self._recovery_rescan_count = 0  # Recovery 재스캔 횟수
        self._open_state = {}         # dict[(node, abs_dir)] = bool - 히스테리시스 이전 상태

        # Wall map visualization
        self._wall_map = WallMapBuilder(grid_size=1.0)
        self._mission_completed = False  # 미션 완료 플래그 (중복 방지)

        self.get_logger().info("AutoControl initialized (Grid-based step control with DFS)")

    # ==================== 서비스/콜백 ====================

    def _handle_start_stop(self, request, response):
        """Start/Stop 서비스 핸들러"""
        if request.start:
            # 새 run 시작: 탐색 메모리 초기화
            self._reset_exploration()
            self._set_state(STATE_IDLE)
            self._start = True
            response.accepted = True
            response.message = "Navigation started"
            self._log("🚀 Navigation started")
        else:
            self._start = False
            self._publish_cmd(0.0, 0.0)
            response.accepted = True
            response.message = "Navigation stopped"
            self._log("🛑 Navigation stopped")
            self._set_state(STATE_IDLE)
        return response

    def _goal_callback(self, msg):
        """목표 좌표 수신"""
        new_goal = (float(msg.x), float(msg.y))

        # 목표 변경 시 탐색 초기화 (새 run으로 간주)
        if self._goal_pose != new_goal:
            self._reset_exploration()
            self._set_state(STATE_IDLE)

        self._goal_pose = new_goal
        msg_str = f"🎯 Goal set: ({msg.x:.1f}, {msg.y:.1f})"
        self._log(msg_str)
        if self._state == STATE_SUCCESS:
            self._set_state(STATE_IDLE)

    def _odom_callback(self, msg):
        """오도메트리 데이터 수신"""
        self._odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self._yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)

    def _scan_callback(self, msg):
        """라이다 스캔 데이터 수신"""
        # 섹터 인덱스 초기화 (첫 실행 시)
        if self._sector_indices is None or len(self._sector_indices[0]) != len(msg.ranges):
            self._sector_indices = self._build_sector_indices(msg)

        # 각 방향의 대표 거리 계산 (퍼센타일 기반)
        self._front_dist, self._front_valid = self._sector_metric(msg.ranges, self._sector_indices[1], SECTOR_PERCENTILE)
        self._left_dist, self._left_valid = self._sector_metric(msg.ranges, self._sector_indices[2], SECTOR_PERCENTILE)
        self._right_dist, self._right_valid = self._sector_metric(msg.ranges, self._sector_indices[3], SECTOR_PERCENTILE)
        self._back_dist, self._back_valid = self._sector_metric(msg.ranges, self._sector_indices[4], SECTOR_PERCENTILE)



    # ==================== 메인 타이머 콜백 ====================

    def _on_timer(self):
        """메인 제어 루프 (20Hz)"""
        # 시작 안 했거나 목표 없으면 정지
        if not self._start or self._goal_pose is None or self._odom_pose is None:
            self._publish_cmd(0.0, 0.0)
            return

        # SUCCESS 상태면 정지 유지
        if self._state == STATE_SUCCESS:
            self._publish_cmd(0.0, 0.0)
            return

        # 목표 도달은 SCAN에서 grid node 기반으로만 체크 (distance 기반 제거)
        # 마지막 DRIVE 완료 후 path edge 올바른 기록 보장

        # 상태머신
        if self._state == STATE_IDLE:
            self._handle_idle()
        elif self._state == STATE_SNAP:
            self._handle_snap()
        elif self._state == STATE_SCAN:
            self._handle_scan()
        elif self._state == STATE_TURN:
            self._handle_turn()
        elif self._state == STATE_DRIVE:
            self._handle_drive()

    # ==================== 상태 핸들러 ====================

    def _handle_idle(self):
        """IDLE 상태: 가까운 교차점으로 스냅 시작"""
        gx, gy = self._snap_to_even_grid(self._odom_pose[0], self._odom_pose[1])
        self._snap_target = (gx, gy)
        self._log(f"SNAP: from ({self._odom_pose[0]:.2f},{self._odom_pose[1]:.2f}) to ({gx:.1f},{gy:.1f})")
        self._set_state(STATE_SNAP)

    def _handle_snap(self):
        """SNAP 상태: 가까운 교차점으로 이동"""
        dist = self._distance(self._odom_pose, self._snap_target)

        # 교차점 도달
        if dist < POSITION_TOLERANCE:
            self._publish_cmd(0.0, 0.0)

            # 현재 노드 업데이트 (정수 인덱스)
            ix = round(self._snap_target[0] / GRID_STEP)
            iy = round(self._snap_target[1] / GRID_STEP)
            self._current_node = (ix, iy)

            # 시작 노드 설정 (최초 1회)
            if self._start_node is None:
                self._start_node = self._current_node
                self._log(f"START node set: {self._current_node}")

            self._log(f"SNAP complete: ({self._snap_target[0]:.1f},{self._snap_target[1]:.1f}) -> node {self._current_node}")
            self._set_state(STATE_SCAN)
            return

        # 교차점으로 이동 (P제어)
        dx = self._snap_target[0] - self._odom_pose[0]
        dy = self._snap_target[1] - self._odom_pose[1]
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self._yaw)

        # 방향이 크게 틀어졌으면 회전
        if abs(yaw_error) > 0.3:
            w = 0.4 if yaw_error > 0 else -0.4
            self._publish_cmd(0.0, w)
        else:
            # 직진 (방향 미세 조정)
            w = max(-0.3, min(0.3, 1.0 * yaw_error))
            self._publish_cmd(0.15, w)

    def _handle_scan(self):
        """SCAN 상태: front/left/right 스캔 후 방향 결정 (back은 Recovery용)"""

        # snapped_yaw 무조건 정의 (UnboundLocalError 방지)
        if self._yaw is not None:
            snapped_yaw = self._snap_yaw_to_cardinal(self._yaw)
        else:
            snapped_yaw = 0.0  # fallback

        # ========== Pre-scan Alignment: Position + Yaw + Settle ==========

        # 1A. 위치 정렬 체크 및 보정
        if not self._prescan_pos_aligned:
            if self._odom_pose is None:
                return

            # 목표 그리드 중심 좌표 계산
            current_x, current_y = self._odom_pose
            target_x = round(current_x / GRID_STEP) * GRID_STEP
            target_y = round(current_y / GRID_STEP) * GRID_STEP

            dx = target_x - current_x
            dy = target_y - current_y
            pos_error = math.sqrt(dx*dx + dy*dy)

            if pos_error > POS_ALIGN_TOLERANCE:
                # 위치 보정: 목표 방향으로 느리게 이동
                target_angle = math.atan2(dy, dx)
                angle_error = self._normalize_angle(target_angle - self._yaw)

                # 목표 방향 회전하며 전진
                v = 0.08 * min(1.0, pos_error / 0.1)  # 거리 비례 속도 (최대 0.08m/s)
                w = 2.0 * angle_error  # 방향 조정
                w = max(-0.5, min(0.5, w))

                self._publish_cmd(v, w)

                # 첫 번째 정렬 시작 시 로그
                if self._state_elapsed() < 0.1:
                    node = self._current_node if self._current_node else (round(current_x), round(current_y))
                    self.get_logger().info(
                        f"[PRESCAN-POS] node={node} pos=({current_x:.3f},{current_y:.3f}) -> "
                        f"target=({target_x:.3f},{target_y:.3f}) pos_err={pos_error:.4f}m"
                    )
                return
            else:
                # 위치 정렬 완료
                self._prescan_pos_aligned = True
                self._publish_cmd(0.0, 0.0)
                node = self._current_node if self._current_node else (round(current_x), round(current_y))
                self.get_logger().info(f"[PRESCAN-POS] ✓ Aligned at node={node} err={pos_error:.4f}m")
                return

        # 1B. Yaw 정렬 체크 및 보정
        if not self._prescan_yaw_aligned:
            snapped_yaw = self._snap_yaw_to_cardinal(self._yaw)
            yaw_error = abs(self._normalize_angle(self._yaw - snapped_yaw))

            if yaw_error > YAW_ALIGN_TOLERANCE:
                # Yaw 보정: 제자리 회전
                angle_diff = self._normalize_angle(snapped_yaw - self._yaw)
                w = 0.3 * (1.0 if angle_diff > 0 else -1.0)  # 느린 회전
                self._publish_cmd(0.0, w)

                # 첫 번째 정렬 시작 시 로그
                if self._state_elapsed() < 0.5 or not hasattr(self, '_yaw_align_logged'):
                    self._yaw_align_logged = True
                    self.get_logger().info(
                        f"[PRESCAN-YAW] yaw={math.degrees(self._yaw):.2f}° -> "
                        f"target={math.degrees(snapped_yaw):.2f}° yaw_err={math.degrees(yaw_error):.3f}°"
                    )
                return
            else:
                # Yaw 정렬 완료
                self._prescan_yaw_aligned = True
                self._prescan_settle_start = time.monotonic()
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    f"[PRESCAN-YAW] ✓ Aligned yaw={math.degrees(self._yaw):.2f}° err={math.degrees(yaw_error):.3f}°"
                )
                self._yaw_align_logged = False
                return

        # 1C. 정렬 후 안정화 대기
        if self._prescan_settle_start is not None:
            settle_elapsed = time.monotonic() - self._prescan_settle_start
            if settle_elapsed < PRESCAN_SETTLE_TIME:
                self._publish_cmd(0.0, 0.0)
                return
            else:
                # 안정화 완료: 최종 정렬 상태 로그
                if self._odom_pose:
                    current_x, current_y = self._odom_pose
                    target_x = round(current_x / GRID_STEP) * GRID_STEP
                    target_y = round(current_y / GRID_STEP) * GRID_STEP
                    final_pos_err = math.sqrt((target_x-current_x)**2 + (target_y-current_y)**2)
                    snapped_yaw = self._snap_yaw_to_cardinal(self._yaw)
                    final_yaw_err = abs(self._normalize_angle(self._yaw - snapped_yaw))
                    node = self._current_node if self._current_node else (round(current_x), round(current_y))

                    self.get_logger().info(
                        f"[PRESCAN] ✓✓ READY node={node} pos=({current_x:.3f},{current_y:.3f}) pos_err={final_pos_err:.4f}m "
                        f"yaw={math.degrees(self._yaw):.2f}° yaw_err={math.degrees(final_yaw_err):.3f}°"
                    )

                self._prescan_settle_start = None  # settle 완료 표시

        # ========== 기존 SCAN 로직 시작 ==========

        # 2. 안정화 기간 (0.2초 정지 + 샘플링)
        elapsed = self._state_elapsed()
        # prescan alignment 시간 고려한 샘플링 시작 시점 조정
        sampling_start_time = PRESCAN_SETTLE_TIME + 0.1

        if elapsed < sampling_start_time + SCAN_STABILIZE_TIME:
            self._publish_cmd(0.0, 0.0)
            if elapsed >= sampling_start_time and len(self._scan_samples) < SCAN_SAMPLE_COUNT:
                self._scan_samples.append((self._front_dist, self._left_dist, self._right_dist, self._back_dist))
            return

        # 3. 안정화 완료: median 계산
        if not self._scan_stabilized and len(self._scan_samples) > 0:
            fronts = sorted([s[0] for s in self._scan_samples])
            lefts = sorted([s[1] for s in self._scan_samples])
            rights = sorted([s[2] for s in self._scan_samples])
            backs = sorted([s[3] for s in self._scan_samples])
            mid_idx = len(self._scan_samples) // 2
            front_dist, left_dist, right_dist, back_dist = fronts[mid_idx], lefts[mid_idx], rights[mid_idx], backs[mid_idx]
            self._scan_stabilized = True
        else:
            front_dist, left_dist, right_dist, back_dist = self._front_dist, self._left_dist, self._right_dist, self._back_dist

        # 4. 현재 방향 및 노드
        self._current_dir = self._yaw_to_dir(self._yaw)
        node = self._current_node
        if node is None:
            self._log("[ERROR] current_node is None in SCAN")
            return

        # 4-1. Goal completion using grid node equality (ONLY trigger)
        goal_node = self._goal_node()
        if goal_node is not None and node == goal_node and not self._mission_completed:
            self._goal_reached_pending = True
            self._log(f"🎯 Goal node {goal_node} reached in SCAN! Final scan will run once then complete.")

        # 5. 4방향 open 판정 (hysteresis 적용)
        # 절대 방향 계산
        front_abs = self._current_dir
        left_abs = (self._current_dir + 3) % 4
        right_abs = (self._current_dir + 1) % 4
        back_abs = (self._current_dir + 2) % 4

        # Hysteresis를 적용한 open/close 판정
        def apply_hysteresis(dist, node, abs_dir):
            key = (node, abs_dir)
            if dist > INTERSECTION_OPEN_THRESH:
                self._open_state[key] = True
                return True
            elif dist < INTERSECTION_CLOSE_THRESH:
                self._open_state[key] = False
                return False
            else:
                # 중간 영역: 이전 상태 유지 (없으면 OPEN_THRESH 기준)
                return self._open_state.get(key, dist > INTERSECTION_OPEN_THRESH)

        front_open = apply_hysteresis(front_dist, node, front_abs)
        left_open = apply_hysteresis(left_dist, node, left_abs)
        right_open = apply_hysteresis(right_dist, node, right_abs)
        back_open = apply_hysteresis(back_dist, node, back_abs)

        # Wall map 업데이트: 절대 방향 기반 open/close 상태 전달
        abs_open_state = {
            front_abs: front_open,
            left_abs: left_open,
            right_abs: right_open,
            back_abs: back_open
        }
        self._wall_map.update_from_scan(node, abs_open_state)

        # 목표 도달 후 마지막 스캔 완료 체크 (경로 결정 전 확인)
        if self._goal_reached_pending:
            self._publish_cmd(0.0, 0.0)
            self._set_state(STATE_SUCCESS)
            self.nav_status_pub.publish(String(data="SUCCESS"))
            if not self._success_published:
                self._log(f"✅ Goal reached! Final scan completed at {self._current_node}")
                self._success_published = True
                # Wall map 렌더링 및 미션 완료 이벤트 퍼블리시
                self._publish_mission_complete(goal_reached=True)
            # Ensure controller stops completely after final scan
            self._goal_reached_pending = False
            self._start = False
            return

        # 샘플 초기화
        self._scan_samples = []
        self._scan_stabilized = False

        # 6. open_dirs 계산: DECIDE용 (F/L/R만), MAP용 (F/L/R + back)
        open_dirs_decide = set()
        if front_open:
            open_dirs_decide.add(front_abs)
        if left_open:
            open_dirs_decide.add(left_abs)
        if right_open:
            open_dirs_decide.add(right_abs)

        # MAP용: decide + back (back_open이면)
        open_dirs_map = open_dirs_decide.copy()
        if back_open:
            open_dirs_map.add(back_abs)

        # 7. discovered에 누적 (MAP 버전 사용)
        self._discovered.setdefault(node, set()).update(open_dirs_map)

        # 8. 디버그 로그 (SCAN 상태 가시화)
        untried = self._discovered[node] - self._tried.get(node, set())
        self.get_logger().info(
            f"[SCAN] node={node} cur_dir={self._current_dir} yaw={math.degrees(self._yaw):.1f}° "
            f"snap={math.degrees(snapped_yaw):.1f}° | "
            f"dist: F={front_dist:.2f} L={left_dist:.2f} R={right_dist:.2f} B={back_dist:.2f} | "
            f"open_rel: F={front_open} L={left_open} R={right_open} B={back_open} | "
            f"open_decide={sorted(open_dirs_decide)} open_map={sorted(open_dirs_map)} | "
            f"disc={sorted(self._discovered[node])} tried={sorted(self._tried.get(node, set()))} untried={sorted(untried)}"
        )

        # 9. Dead-end 판정 (F/L/R 막힘 + back 열림 + parent 존재)
        all_blocked = not front_open and not left_open and not right_open
        dead_end = all_blocked and back_open and (node in self._parent_dir)

        if dead_end:
            # Dead-end 정상 백트래킹 처리 (Recovery 아님)
            self.get_logger().info(f"[DEAD-END] node={node} back_open={back_open} → normal backtrack")
            self._recovery_rescan_count = 0
            chosen_dir = self._parent_dir[node]
            # tried 기록 (백트래킹도 tried 표시)
            self._tried.setdefault(node, set()).add(chosen_dir)
            is_backtrack = True
        elif all_blocked:
            # 10. Recovery: 진짜 막힘 (valid_count도 체크)
            low_confidence = (self._front_valid < 10 or self._left_valid < 10 or self._right_valid < 10)
            self.get_logger().info(
                f"[RECOVERY] all_blocked=True (not dead-end) rescan_count={self._recovery_rescan_count} "
                f"valid=(F:{self._front_valid} L:{self._left_valid} R:{self._right_valid} B:{self._back_valid}) "
                f"back_open={back_open} dead_end={dead_end}"
            )
            if self._recovery_rescan_count < 3 or low_confidence:
                # 재스캔 3회 시도 (또는 valid_count 낮으면 추가)
                self._recovery_rescan_count += 1
                self._log(f"[Recovery] Rescan attempt {self._recovery_rescan_count}")
                return
            else:
                # 재스캔해도 막힘
                self._recovery_rescan_count = 0
                if node in self._parent_dir:
                    chosen_dir = self._parent_dir[node]
                    self.get_logger().info(f"[RECOVERY] Moving to parent via dir {chosen_dir}")
                else:
                    # parent도 없으면 U-turn
                    if back_open:
                        chosen_dir = back_abs
                        self.get_logger().info(f"[RECOVERY] U-turn to dir {chosen_dir}")
                    else:
                        self.get_logger().info("[RECOVERY] Completely blocked, stopping")
                        self._publish_cmd(0.0, 0.0)
                        return
                # tried 기록
                self._tried.setdefault(node, set()).add(chosen_dir)
                is_backtrack = True
        else:
            # 11. 정상 경로 결정
            self._recovery_rescan_count = 0

            chosen_dir, is_backtrack = self._choose_next_dir(node, open_dirs_decide, back_open)

            if chosen_dir is None:
                self._log("[DECIDE] Exploration complete")
                self._exploration_done = True
                self._publish_cmd(0.0, 0.0)
                return

        self._chosen_dir = chosen_dir

        # 10. 회전 또는 전진
        target_yaw = self._dir_to_yaw(chosen_dir)
        yaw_diff = abs(self._normalize_angle(target_yaw - self._yaw))
        if yaw_diff > YAW_TOLERANCE:
            self._target_yaw = target_yaw
            self._log(f"TURN to dir={chosen_dir} ({math.degrees(target_yaw):.0f}°) {'[BACKTRACK]' if is_backtrack else ''}")
            self._set_state(STATE_TURN)
        else:
            next_node = self._neighbor(node, chosen_dir)
            next_x, next_y = next_node[0] * GRID_STEP, next_node[1] * GRID_STEP
            self._drive_target = (next_x, next_y)
            self._log(f"DRIVE to {next_node} {'[BACKTRACK]' if is_backtrack else ''}")
            self._set_state(STATE_DRIVE)

    def _handle_turn(self):
        """TURN 상태: 목표 yaw까지 회전"""
        yaw_error = self._normalize_angle(self._target_yaw - self._yaw)

        # 회전 완료
        if abs(yaw_error) < YAW_TOLERANCE:
            self._publish_cmd(0.0, 0.0)
            self._log(f"TURN complete: {math.degrees(self._yaw):.1f}°")

            # 회전 후 DRIVE (chosen_dir 이동)
            if self._chosen_dir is not None:
                next_node = self._neighbor(self._current_node, self._chosen_dir)
                next_x = next_node[0] * GRID_STEP
                next_y = next_node[1] * GRID_STEP
                self._drive_target = (next_x, next_y)
                self._set_state(STATE_DRIVE)
            else:
                # chosen_dir이 없으면 SCAN으로
                self._set_state(STATE_SCAN)
            return

        # 회전 (더 짧은 방향으로)
        w = W_TURN if yaw_error > 0 else -W_TURN
        self._publish_cmd(0.0, w)

    def _handle_drive(self):
        """DRIVE 상태: 다음 교차점까지 1m 전진"""
        # 긴급 정지: 전방 장애물
        if self._front_dist < STOP_MARGIN:
            self._publish_cmd(0.0, 0.0)
            self._log(f"STOP: front blocked {self._front_dist:.2f}m < {STOP_MARGIN}m")
            self._set_state(STATE_SCAN)
            return

        # 목표 교차점 도달 체크
        dist = self._distance(self._odom_pose, self._drive_target)
        if dist < POSITION_TOLERANCE:
            self._publish_cmd(0.0, 0.0)

            # 이전 노드
            old_node = self._current_node

            # 다음 노드 계산
            next_node = self._neighbor(old_node, self._chosen_dir)

            # ========== discovered 확정 기록 ==========
            # 방금 이동 방향을 discovered[old_node]에 확정 추가
            if old_node not in self._discovered:
                self._discovered[old_node] = set()
            self._discovered[old_node].add(self._chosen_dir)
            self._log(f"DFS: discovered[{old_node}] confirmed dir {self._chosen_dir}")

            # parent 정보 업데이트 (처음 방문하는 노드만)
            if next_node not in self._parent:
                self._parent[next_node] = old_node
                self._parent_dir[next_node] = (self._chosen_dir + 2) % 4  # 반대 방향
                self._log(f"DFS: parent[{next_node}] = {old_node}, parent_dir = {self._parent_dir[next_node]}")

                # 새 노드 도착 시 들어온 방향을 tried 초기화 (Item 4A)
                incoming_parent_dir = self._parent_dir[next_node]
                self._tried.setdefault(next_node, set()).add(incoming_parent_dir)
                self._log(f"DFS: tried[{next_node}] initialized with incoming dir {incoming_parent_dir}")

            # 현재 노드 업데이트
            self._current_node = next_node

            # Wall map에 경로 에지 기록
            self._wall_map.update_edge(old_node, next_node)

            self._log(f"DRIVE complete: node {self._current_node}")
            self._set_state(STATE_SCAN)
            return

        # 직진 (방향 유지)
        dx = self._drive_target[0] - self._odom_pose[0]
        dy = self._drive_target[1] - self._odom_pose[1]
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self._yaw)

        # 방향 미세 조정하며 전진
        w = max(-0.5, min(0.5, 2.0 * yaw_error))
        self._publish_cmd(V_DRIVE, w)

    # ==================== 헬퍼 함수 ====================

    def _reset_exploration(self):
        """새 run 초기화: DFS 탐색 메모리 및 주행 타겟 초기화"""
        # DFS 탐색 상태 초기화
        self._discovered = {}
        self._tried = {}
        self._parent = {}
        self._parent_dir = {}
        self._current_node = None
        self._current_dir = None
        self._chosen_dir = None
        self._start_node = None
        self._backtrack_target = None
        self._exploration_done = False
        self._recovery_rescan_count = 0
        # Wall map 초기화
        self._wall_map = WallMapBuilder(grid_size=1.0)
        self._mission_completed = False
        self._success_published = False
        self._goal_reached_pending = False
        self._open_state = {}  # Hysteresis 상태 초기화

        # 주행 타겟/상태 초기화
        self._snap_target = None
        self._drive_target = None
        self._target_yaw = None
        self._scan_samples = []
        self._scan_stabilized = False

        self.get_logger().info("[RESET] Exploration memory cleared")

    def _snap_to_even_grid(self, x, y):
        """가장 가까운 정수 교차점으로 스냅 (1m 간격)"""
        gx = round(x / EVEN_SNAP_INTERVAL) * EVEN_SNAP_INTERVAL
        gy = round(y / EVEN_SNAP_INTERVAL) * EVEN_SNAP_INTERVAL
        return (gx, gy)

    def _snap_yaw_to_cardinal(self, yaw):
        """가장 가까운 90도 각도로 스냅 (0, π/2, π, -π/2)"""
        # 4방향: 0(동), π/2(북), π(서), -π/2(남)
        cardinals = [0.0, math.pi/2, math.pi, -math.pi/2]
        min_diff = math.inf
        snapped = 0.0
        for cardinal in cardinals:
            diff = abs(self._normalize_angle(yaw - cardinal))
            if diff < min_diff:
                min_diff = diff
                snapped = cardinal
        return snapped

    def _yaw_to_dir(self, yaw):
        """Yaw 각도를 방향 인덱스로 변환
        0=North(+y), 1=East(+x), 2=South(-y), 3=West(-x)
        """
        # yaw = π/2 → dir 0 (North)
        # yaw = 0 → dir 1 (East)
        # yaw = -π/2 → dir 2 (South)
        # yaw = π (or -π) → dir 3 (West)
        angle = self._normalize_angle(yaw)
        if -math.pi/4 <= angle < math.pi/4:
            return 1  # East
        elif math.pi/4 <= angle < 3*math.pi/4:
            return 0  # North
        elif angle >= 3*math.pi/4 or angle < -3*math.pi/4:
            return 3  # West
        else:  # -3*math.pi/4 <= angle < -math.pi/4
            return 2  # South

    def _dir_to_yaw(self, direction):
        """방향 인덱스를 Yaw 각도로 변환
        0→π/2(North), 1→0(East), 2→-π/2(South), 3→π(West)
        """
        if direction == 0:
            return math.pi / 2  # North
        elif direction == 1:
            return 0.0  # East
        elif direction == 2:
            return -math.pi / 2  # South
        elif direction == 3:
            return math.pi  # West
        else:
            self.get_logger().error(f"Invalid direction: {direction}")
            return 0.0

    def _get_next_node(self, node, direction):
        """현재 노드에서 방향으로 이동한 다음 노드 좌표 계산
        node = (ix, iy), direction = 0~3
        """
        ix, iy = node
        if direction == 0:  # North (+y)
            return (ix, iy + 1)
        elif direction == 1:  # East (+x)
            return (ix + 1, iy)
        elif direction == 2:  # South (-y)
            return (ix, iy - 1)
        elif direction == 3:  # West (-x)
            return (ix - 1, iy)
        else:
            self.get_logger().error(f"Invalid direction: {direction}")
            return node

    def _choose_next_dir(self, node, open_dirs, back_open):
        """목표 직선상 우선 + 미해결 분기점 백트래킹 DFS 로직
        Args:
            node: 현재 노드 (ix, iy)
            open_dirs: 현재 교차점에서 열린 방향들 (set of 0~3, F/L/R만)
            back_open: 후방이 열려있는지 (bool)
        Returns:
            (chosen_dir, is_backtrack)
        """
        # 1. tried 초기화
        if node not in self._tried:
            self._tried[node] = set()

        # 2. untried 후보 계산 (F/L/R만)
        untried = (self._discovered[node] - self._tried[node]) & open_dirs

        # 3. 목표 직선상 우선 로직 (A)
        goal_node = self._goal_node()
        desired_dir = None
        goal_on_line = False

        if goal_node is not None:
            ix, iy = node
            gx, gy = goal_node

            # 같은 열 (x 같음): North or South
            if gx == ix and gy != iy:
                desired_dir = 0 if gy > iy else 2  # North if gy>iy else South
                goal_on_line = True
            # 같은 행 (y 같음): East or West
            elif gy == iy and gx != ix:
                desired_dir = 1 if gx > ix else 3  # East if gx>ix else West
                goal_on_line = True

        if goal_on_line and desired_dir is not None:
            # desired_dir이 untried에 있고 open_dirs에 있으면 무조건 선택
            if desired_dir in untried:
                self._tried.setdefault(node, set()).add(desired_dir)
                self.get_logger().info(
                    f"[GOAL-LINE] node={node} goal_node={goal_node} desired_dir={desired_dir} "
                    f"back_open={back_open} open_dirs={sorted(open_dirs)} untried={sorted(untried)} chosen={desired_dir}"
                )
                return (desired_dir, False)

            # (예외) desired_dir이 뒤(180도)이고, open_dirs엔 없지만 back_open=True이고 untried 해당
            opposite_current = (self._current_dir + 2) % 4
            if desired_dir == opposite_current and desired_dir not in open_dirs and back_open:
                # discovered 없지만 back으로 갈 수 있는지 확인
                if desired_dir not in self._tried.get(node, set()):
                    self._tried.setdefault(node, set()).add(desired_dir)
                    self.get_logger().info(
                        f"[GOAL-LINE] node={node} goal_node={goal_node} desired_dir={desired_dir}(BACK) "
                        f"back_open=True open_dirs={sorted(open_dirs)} untried={sorted(untried)} chosen={desired_dir}"
                    )
                    # back 이동 시 discovered 추가
                    self._discovered[node].add(desired_dir)
                    return (desired_dir, False)

        # 4. 백트래킹 모드 체크

        # 5. 백트래킹 모드 체크
        if self._backtrack_target is not None:
            # 백트래킹 중
            if node == self._backtrack_target:
                # 목표 분기점 도착
                self.get_logger().info(f"[DECIDE] Reached backtrack_target {node}")
                self._backtrack_target = None
                # 도착 후 재판단 (재귀 방지용 untried 재확인)
                if untried:
                    # 미해결 출구 있음
                    goal = self._goal_node()
                    chosen = self._choose_closest_to_goal(list(untried), node, goal)
                    self._tried.setdefault(node, set()).add(chosen)
                    self.get_logger().info(
                        f"[DECIDE] node={node} open={sorted(open_dirs)} disc={sorted(self._discovered[node])} "
                        f"tried={sorted(self._tried[node])} untried={sorted(untried)} chosen={chosen} back=False backtrack_target=None"
                    )
                    return (chosen, False)
                else:
                    # 도착했는데도 untried 없으면 계속 백트래킹
                    pass  # 아래 백트래킹 로직 계속
            else:
                # 아직 목표에 도달하지 않음: untried가 생겼는지 체크
                if untried:
                    # 백트래킹 중 새로운 untried 발견 → 즉시 탐색 전환
                    self.get_logger().info(f"[DECIDE] Backtrack interrupted at {node}: untried found")
                    self._backtrack_target = None
                    goal = self._goal_node()
                    chosen = self._choose_closest_to_goal(list(untried), node, goal)
                    self._tried.setdefault(node, set()).add(chosen)
                    self.get_logger().info(
                        f"[DECIDE] node={node} open={sorted(open_dirs)} disc={sorted(self._discovered[node])} "
                        f"tried={sorted(self._tried[node])} untried={sorted(untried)} chosen={chosen} back=False backtrack_target=None"
                    )
                    return (chosen, False)
                else:
                    # 계속 백트래킹
                    if node in self._parent_dir:
                        chosen = self._parent_dir[node]
                        self._tried.setdefault(node, set()).add(chosen)
                        self.get_logger().info(
                            f"[DECIDE] node={node} open={sorted(open_dirs)} disc={sorted(self._discovered[node])} "
                            f"tried={sorted(self._tried[node])} untried=[] chosen={chosen} back=True backtrack_target={self._backtrack_target}"
                        )
                        return (chosen, True)
                    else:
                        self.get_logger().error(f"[DECIDE] Backtracking failed: no parent_dir at {node}")
                        return (None, True)

        # 6. 정상 탐색 모드 (B)
        if untried:
            # 미시도 경로 존재 → goal-bias 선택
            chosen = self._choose_closest_to_goal(list(untried), node, goal_node)
            self._tried.setdefault(node, set()).add(chosen)
            self.get_logger().info(
                f"[DECIDE] node={node} open={sorted(open_dirs)} disc={sorted(self._discovered[node])} "
                f"tried={sorted(self._tried[node])} untried={sorted(untried)} chosen={chosen} back=False backtrack_target=None"
            )
            return (chosen, False)
        else:
            # 7. untried 없음 → 미해결 분기점 찾기
            backtrack_target = self._find_unresolved_ancestor(node)
            if backtrack_target is None:
                # 미해결 분기점 없음 → 탐색 완료
                self.get_logger().info(f"[DECIDE] node={node} No unresolved ancestor → Exploration complete")
                return (None, True)
            else:
                # 미해결 분기점 발견 → 백트래킹 시작
                self._backtrack_target = backtrack_target
                if node in self._parent_dir:
                    chosen = self._parent_dir[node]
                    self._tried.setdefault(node, set()).add(chosen)
                    self.get_logger().info(
                        f"[DECIDE] node={node} open={sorted(open_dirs)} disc={sorted(self._discovered[node])} "
                        f"tried={sorted(self._tried[node])} untried=[] chosen={chosen} back=True backtrack_target={backtrack_target}"
                    )
                    return (chosen, True)
                else:
                    self.get_logger().error(f"[DECIDE] Cannot backtrack: no parent_dir at {node}")
                    return (None, True)

    def _find_unresolved_ancestor(self, node):
        """parent 체인을 따라 올라가며 첫 번째 미해결 분기점 찾기
        Returns:
            미해결 분기점 노드, 없으면 None
        """
        current = node
        visited_chain = set([current])  # 무한루프 방지
        while current in self._parent:
            parent = self._parent[current]
            if parent in visited_chain:
                self.get_logger().warn(f"[DECIDE] Loop detected in parent chain at {parent}")
                break
            visited_chain.add(parent)

            # parent가 미해결인지 체크 (tried가 없어도 discovered만 있으면 미해결 가능)
            tried_set = self._tried.get(parent, set())
            disc_set = self._discovered.get(parent, set())
            untried_at_parent = disc_set - tried_set

            if untried_at_parent:
                # 미해결 분기점 발견
                return parent
            current = parent
        return None

    def _choose_closest_to_goal(self, directions, node, goal_node):
        """방향들 중 goal에 가장 가까워지는 방향 선택"""
        if not directions:
            return -1
        if goal_node is None:
            return directions[0]

        best_dir = directions[0]
        best_dist = self._node_dist(self._neighbor(node, best_dir), goal_node)
        for d in directions[1:]:
            next_node = self._neighbor(node, d)
            dist = self._node_dist(next_node, goal_node)
            if dist < best_dist:
                best_dist = dist
                best_dir = d
        return best_dir

    def _distance(self, pos1, pos2):
        """두 점 사이 거리"""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx*dx + dy*dy)

    def _goal_node(self):
        """목표 좌표를 그리드 노드 인덱스로 변환"""
        if self._goal_pose is None:
            return None
        gx = round(self._goal_pose[0] / GRID_STEP)
        gy = round(self._goal_pose[1] / GRID_STEP)
        return (gx, gy)

    def _neighbor(self, node, direction):
        """노드에서 방향으로 이동한 이웃 노드 반환
        direction: 0=N(+y), 1=E(+x), 2=S(-y), 3=W(-x)
        """
        ix, iy = node
        if direction == 0:  # North
            return (ix, iy + 1)
        elif direction == 1:  # East
            return (ix + 1, iy)
        elif direction == 2:  # South
            return (ix, iy - 1)
        elif direction == 3:  # West
            return (ix - 1, iy)
        else:
            return node

    def _node_dist(self, node_a, node_b):
        """두 노드 간 유클리드 거리 (그리드 인덱스 기준)"""
        if node_a is None or node_b is None:
            return math.inf
        dx = node_b[0] - node_a[0]
        dy = node_b[1] - node_a[1]
        return math.sqrt(dx*dx + dy*dy)

    def _normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def _yaw_from_quaternion(self, quat):
        """쿼터니언에서 yaw 추출"""
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)


    # ==================== 라이다 섹터 처리 ====================

    def _build_sector_indices(self, msg):
        """라이다 각도별 인덱스 생성 (각도 정규화 적용)"""
        angles = []
        for i in range(len(msg.ranges)):
            raw_angle = msg.angle_min + (i * msg.angle_increment)
            # 각도 -π~π 범위 정규화
            normalized_angle = self._normalize_angle(raw_angle)
            angles.append(normalized_angle)

        front_indices = self._indices_for_sector(angles, FRONT_MIN_DEG, FRONT_MAX_DEG)
        left_indices = self._indices_for_sector(angles, LEFT_MIN_DEG, LEFT_MAX_DEG)
        right_indices = self._indices_for_sector(angles, RIGHT_MIN_DEG, RIGHT_MAX_DEG)

        # 후방 섹터 두 구간 합쳨 (150~180°, -180~-150°)
        back1_indices = self._indices_for_sector(angles, BACK1_MIN_DEG, BACK1_MAX_DEG)
        back2_indices = self._indices_for_sector(angles, BACK2_MIN_DEG, BACK2_MAX_DEG)
        back_indices = back1_indices + back2_indices

        return (angles, front_indices, left_indices, right_indices, back_indices)

    def _indices_for_sector(self, angles, min_deg, max_deg):
        """특정 각도 범위 인덱스 필터링 (정규화 각도 -180~180 사용)"""
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)

        # 정규화 각도 항상 -π~π 범위로 단순 비교
        return [i for i, angle in enumerate(angles)
                if min_rad <= angle <= max_rad]

    def _sector_metric(self, ranges, indices, percentile=50):
        """섹터 내 대표 거리 계산 (퍼센타일 기반)

        Args:
            ranges: LaserScan ranges 데이터
            indices: 섹터 인덱스 리스트
            percentile: 사용할 퍼센타일 (기본 50% = median)

        Returns:
            (distance, valid_count): 대표 거리와 유효 데이터 개수
        """
        if not indices:
            # 섹터 비어있으면 (0.0, 0) 반환 (막힘 판정)
            return (0.0, 0)

        values = []
        for idx in indices:
            value = ranges[idx]
            if math.isfinite(value) and value > 0.0:
                values.append(value)

        if not values:
            # 유효 값 없으면 (0.0, 0) 반환 (막힘 판정)
            return (0.0, 0)

        # 퍼센타일 계산
        values.sort()
        index = int(len(values) * percentile / 100.0)
        index = max(0, min(index, len(values) - 1))  # 범위 제한

        return (values[index], len(values))

    # ==================== 유틸리티 ====================

    def _state_elapsed(self):
        """현재 상태 경과 시간 반환 (초)"""
        return time.monotonic() - self._state_start_time

    def _set_state(self, state):
        """상태 변경 및 로깅"""
        if self._state == state:
            return
        self._state = state
        self._state_start_time = time.monotonic()
        self.get_logger().info(f"State: {state}")

        # SCAN 상태 진입 시 pre-scan alignment 플래그 초기화
        if state == STATE_SCAN:
            self._prescan_pos_aligned = False
            self._prescan_yaw_aligned = False
            self._prescan_settle_start = None
            if hasattr(self, '_yaw_align_logged'):
                self._yaw_align_logged = False

    def _log(self, message):
        """로그 메시지 퍼블리시"""
        self.log_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _publish_cmd(self, linear, angular):
        """cmd_vel 퍼블리시"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _publish_mission_complete(self, goal_reached: bool = True):
        """미션 완료 시 wall map PNG 렌더링 및 GUI 이벤트 퍼블리시"""
        if self._mission_completed:
            self.get_logger().info("[MISSION] Already completed, skipping")
            return  # 중복 방지
        self._mission_completed = True

        self.get_logger().info(f"[MISSION] ✅ Mission complete! goal_reached={goal_reached}")
        self.get_logger().info(f"[MISSION] Wall segments: {len(self._wall_map.wall_segments)}")
        self.get_logger().info(f"[MISSION] Path edges: {len(self._wall_map.path_edges)}")
        self.get_logger().info(f"[MISSION] Start node: {self._start_node}, Current node: {self._current_node}")

        # Wall map PNG 렌더링
        filepath = os.path.expanduser("~/.ros/maze_walls.png")
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        goal_node = None
        if self._goal_pose:
            gx = round(self._goal_pose[0] / GRID_STEP)
            gy = round(self._goal_pose[1] / GRID_STEP)
            goal_node = (gx, gy)
            self.get_logger().info(f"[MISSION] Goal node: {goal_node}")

        self.get_logger().info(f"[MISSION] Rendering wall map to {filepath}...")
        self._wall_map.render_png(
            filepath,
            start_node=self._start_node,
            goal_node=goal_node,
            current_node=self._current_node
        )

        # 파일 생성 확인
        if os.path.exists(filepath):
            file_size = os.path.getsize(filepath)
            self.get_logger().info(f"[MISSION] ✅ Wall map rendered successfully: {filepath} ({file_size} bytes)")
        else:
            self.get_logger().error(f"[MISSION] ❌ ERROR: Wall map file not created at {filepath}")

        # GUI 이벤트 퍼블리시
        event_msg = {
            "event": "mission_done",
            "map_path": filepath,
            "goal_reached": goal_reached
        }
        self.mission_event_pub.publish(String(data=json.dumps(event_msg)))
        self.get_logger().info(f"[MISSION] 📡 Event published to /mission_event: {event_msg}")


def main(args=None):
    rclpy.init(args=args)
    auto_control_node = AutoControl()
    try:
        rclpy.spin(auto_control_node)
    except KeyboardInterrupt:
        auto_control_node.get_logger().info('Keyboard interrupt!')
    finally:
        auto_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
