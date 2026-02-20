"""
벽 기반 미로 지도 시각화 (SLAM 아님)
격자 기반 탐색 중 감지된 벽 세그먼트 추적
"""
import os
import math
from typing import Set, Tuple, Dict, List, Optional


class WallMapBuilder:
    """격자 교차점에서 감지된 벽 세그먼트로 미로 맵 구축"""

    def __init__(self, grid_size: float = 1.0):
        self.grid_size = grid_size
        self.wall_segments: Set[Tuple[Tuple[float, float], Tuple[float, float]]] = set()
        self.path_edges: List[Tuple[Tuple[int, int], Tuple[int, int]]] = []

    def _canonical_segment(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """끝점을 소수점 3자리로 반올림하고 정렬하여 정규 표현 생성"""
        x1, y1 = round(p1[0], 3), round(p1[1], 3)
        x2, y2 = round(p2[0], 3), round(p2[1], 3)
        if (x1, y1) <= (x2, y2):
            return ((x1, y1), (x2, y2))
        else:
            return ((x2, y2), (x1, y1))

    def add_wall_segment(self, i: int, j: int, abs_dir: int):
        """
        그리드 노드 (i,j)의 절대 방향 abs_dir에 벽 세그먼트 추가
        abs_dir: 0=북(+y), 1=동(+x), 2=남(-y), 3=서(-x)
        벽은 노드 중심에서 ±0.5*grid_size 위치에 배치
        """
        g = self.grid_size
        x = i * g
        y = j * g
        half_g = 0.5 * g

        if abs_dir == 0:  # 북쪽 벽
            seg = self._canonical_segment((x - half_g, y + half_g), (x + half_g, y + half_g))
        elif abs_dir == 1:  # 동쪽 벽
            seg = self._canonical_segment((x + half_g, y - half_g), (x + half_g, y + half_g))
        elif abs_dir == 2:  # 남쪽 벽
            seg = self._canonical_segment((x - half_g, y - half_g), (x + half_g, y - half_g))
        elif abs_dir == 3:  # 서쪽 벽
            seg = self._canonical_segment((x - half_g, y - half_g), (x - half_g, y + half_g))
        else:
            return

        self.wall_segments.add(seg)

    def update_from_scan(self, node: Tuple[int, int], abs_open_state: Dict[int, bool]):
        """
        노드에서의 스캔 결과로 벽 맵 업데이트
        abs_open_state: {abs_dir: is_open} 방향 0,1,2,3
        방향이 닫혀있으면 벽 세그먼트 추가
        """
        i, j = node
        for abs_dir in range(4):
            if abs_dir in abs_open_state:
                is_open = abs_open_state[abs_dir]
                if not is_open:
                    self.add_wall_segment(i, j, abs_dir)

    def update_edge(self, src_node: Tuple[int, int], dst_node: Tuple[int, int]):
        """경로 엣지 기록 (선택사항, 경로 오버레이 시각화용)"""
        self.path_edges.append((src_node, dst_node))

    def render_png(self,
                   filepath: str,
                   start_node: Optional[Tuple[int, int]] = None,
                   goal_node: Optional[Tuple[int, int]] = None,
                   current_node: Optional[Tuple[int, int]] = None):
        """
        벽 맵을 matplotlib 사용하여 PNG 이미지로 렌더링
        """
        print(f"[WallMap] render_png called: filepath={filepath}")
        print(f"[WallMap] Wall segments: {len(self.wall_segments)}")
        print(f"[WallMap] Path edges: {len(self.path_edges)}")
        print(f"[WallMap] start_node={start_node}, goal_node={goal_node}, current_node={current_node}")

        try:
            import matplotlib
            matplotlib.use('Agg')  # 비대화형 백엔드
            import matplotlib.pyplot as plt
            from matplotlib.patches import Circle
            import matplotlib.lines as mlines
        except ImportError:
            print("ERROR: matplotlib not available for rendering")
            return

        # Expand filepath
        filepath = os.path.expanduser(filepath)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        fig, ax = plt.subplots(figsize=(10, 10))

        # 벽 세그먼트 그리기 (굵은 검은 선)
        for (p1, p2) in self.wall_segments:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-', linewidth=3)

        # 경로 엣지 그리기 (얇은 회색 선) - 선택사항
        if self.path_edges:
            for (src, dst) in self.path_edges:
                sx, sy = src[0] * self.grid_size, src[1] * self.grid_size
                dx, dy = dst[0] * self.grid_size, dst[1] * self.grid_size
                ax.plot([sx, dx], [sy, dy], 'gray', linewidth=1, alpha=0.5)

        # 특수 노드 마킹
        g = self.grid_size
        if start_node:
            sx, sy = start_node[0] * g, start_node[1] * g
            ax.add_patch(Circle((sx, sy), 0.15, color='blue', zorder=10))
            ax.text(sx, sy - 0.3, 'START', ha='center', fontsize=10, color='blue', weight='bold')

        if goal_node:
            gx, gy = goal_node[0] * g, goal_node[1] * g
            ax.add_patch(Circle((gx, gy), 0.15, color='red', zorder=10))
            ax.text(gx, gy - 0.3, 'GOAL', ha='center', fontsize=10, color='red', weight='bold')

        # 동일 비율 설정 및 여백 추가
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
        ax.set_xlabel('X (meters)', fontsize=12)
        ax.set_ylabel('Y (meters)', fontsize=12)
        ax.set_title('Maze Wall Map', fontsize=14, weight='bold')

        # 여백과 함께 자동 경계 조정
        if self.wall_segments:
            all_x = [p[0] for seg in self.wall_segments for p in seg]
            all_y = [p[1] for seg in self.wall_segments for p in seg]
            margin = 1.0
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

        plt.tight_layout()
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)

        # 파일 생성 확인
        if os.path.exists(filepath):
            file_size = os.path.getsize(filepath)
            print(f"[WallMap] ✅ Rendered maze map to {filepath} (size: {file_size} bytes)")
        else:
            print(f"[WallMap] ❌ ERROR: File not created at {filepath}")
