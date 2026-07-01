"""nav_overlay.py 鈥?Map overlay drawing utility for navigation demos.

Provides a single function draw_nav_overlay() that draws a 2D map panel and
info text onto a video frame. Used by recording scripts and live visualizers.
"""
import math
import numpy as np
import cv2


def draw_nav_overlay(
    frame: np.ndarray,
    grid: np.ndarray,
    robot_pos: tuple,
    goal: tuple,
    lidar_pts: np.ndarray,
    trail: list,
    global_path: list,
    nav_health: dict,
    elapsed: float,
    scan_count: int,
    map_resolution: float = 0.2,
    map_size: int = 200,
    panel_size: int = 280,
    center_offset: tuple = (3, 1.5),
) -> np.ndarray:
    """Draw a 2D map panel and info text overlay onto a BGR video frame.

    The panel is placed on the right side of the frame. Info text is drawn in
    the top-left corner of the main frame. Modifies the frame in-place and
    also returns it.

    Args:
        frame:          BGR frame to draw on (modified in-place).
        grid:           200x200 occupancy grid (float32, values 0-100).
        robot_pos:      (x, y) world position of the robot.
        goal:           (x, y) world position of the goal.
        lidar_pts:      Nx3 point cloud array, or None.
        trail:          List of (x, y) world-coord tuples for the robot trail.
        global_path:    List of np.ndarray waypoints from nav.mission._path.
        nav_health:     Dict from nav.mission.health()["navigation"].
        elapsed:        Seconds elapsed since navigation start.
        scan_count:     Total number of LiDAR scans received.
        map_resolution: Grid cell size in meters (default 0.2).
        map_size:       Grid dimension in cells (default 200).
        panel_size:     Side length of the square overlay panel in pixels (default 280).
        center_offset:  World (x, y) coords to center the map view on (default (3, 1.5)).

    Returns:
        The same frame (also modified in-place).
    """
    H, W = frame.shape[:2]
    rx, ry = robot_pos
    gx_goal, gy_goal = goal
    cx_off, cy_off = center_offset

    # --- Panel geometry ---------------------------------------------------
    panel_x = W - panel_size - 8   # 8px right margin
    panel_y = 8                     # 8px top margin

    # Scale: how many pixels per meter inside the panel
    # Show map_size * map_resolution metres total; shrink slightly for margin
    world_span = map_size * map_resolution          # e.g. 40m
    scale = (panel_size - 10) / world_span          # px / m

    def w2p(wx, wy):
        """World coord 鈫?panel-local pixel coord."""
        px = int((panel_size // 2) + (wx - cx_off) * scale)
        py = int((panel_size // 2) - (wy - cy_off) * scale)
        return px, py

    def p2frame(px, py):
        """Panel-local pixel 鈫?frame pixel coord."""
        return panel_x + px, panel_y + py

    # --- Semi-transparent dark background panel ---------------------------
    overlay = frame.copy()
    cv2.rectangle(overlay,
                  (panel_x, panel_y),
                  (panel_x + panel_size, panel_y + panel_size),
                  (15, 15, 20), -1)
    cv2.addWeighted(overlay, 0.78, frame, 0.22, 0, frame)

    # Panel border
    cv2.rectangle(frame,
                  (panel_x, panel_y),
                  (panel_x + panel_size, panel_y + panel_size),
                  (60, 60, 80), 1)

    # --- Grid cells -------------------------------------------------------
    half = map_size // 2
    for cell_y in range(0, map_size, 2):          # step 2 for speed
        for cell_x in range(0, map_size, 2):
            v = grid[cell_y, cell_x]
            if v <= 10:
                continue
            wx = (cell_x - half) * map_resolution
            wy = (cell_y - half) * map_resolution
            ppx, ppy = w2p(wx, wy)
            fx, fy = p2frame(ppx, ppy)
            if 0 <= fx < W and 0 <= fy < H:
                # dark red 鈫?bright red as occupancy rises
                intensity = min(255, int(v * 2.5))
                color = (intensity // 4, intensity // 8, intensity)
                cv2.circle(frame, (fx, fy), 2, color, -1)

    # --- LiDAR points -----------------------------------------------------
    if lidar_pts is not None and len(lidar_pts) > 0:
        for p in lidar_pts[::5]:                  # subsample every 5th point
            ppx, ppy = w2p(p[0], p[1])
            fx, fy = p2frame(ppx, ppy)
            if 0 <= fx < W and 0 <= fy < H:
                cv2.circle(frame, (fx, fy), 1, (0, 200, 0), -1)

    # --- Trail ------------------------------------------------------------
    if len(trail) >= 2:
        for i in range(1, len(trail)):
            p1x, p1y = w2p(trail[i - 1][0], trail[i - 1][1])
            p2x, p2y = w2p(trail[i][0], trail[i][1])
            f1 = p2frame(p1x, p1y)
            f2 = p2frame(p2x, p2y)
            # Only draw if both ends are inside the panel area
            if (0 <= p1x < panel_size and 0 <= p1y < panel_size and
                    0 <= p2x < panel_size and 0 <= p2y < panel_size):
                cv2.line(frame, f1, f2, (30, 130, 220), 1)

    # --- Global path (cyan) -----------------------------------------------
    if global_path and len(global_path) >= 2:
        prev_pp = None
        for wp in global_path:
            if hasattr(wp, "__len__") and len(wp) >= 2:
                ppx, ppy = w2p(float(wp[0]), float(wp[1]))
                if 0 <= ppx < panel_size and 0 <= ppy < panel_size:
                    fp = p2frame(ppx, ppy)
                    if prev_pp is not None:
                        cv2.line(frame, prev_pp, fp, (220, 210, 0), 1)
                    prev_pp = fp
                else:
                    prev_pp = None

    # --- Robot (filled orange circle with white border) -------------------
    rppx, rppy = w2p(rx, ry)
    rfx, rfy = p2frame(rppx, rppy)
    if 0 <= rppx < panel_size and 0 <= rppy < panel_size:
        cv2.circle(frame, (rfx, rfy), 6, (0, 165, 255), -1)
        cv2.circle(frame, (rfx, rfy), 6, (255, 255, 255), 1)

    # --- Goal (red star marker) ------------------------------------------
    gppx, gppy = w2p(gx_goal, gy_goal)
    gfx, gfy = p2frame(gppx, gppy)
    if 0 <= gppx < panel_size and 0 <= gppy < panel_size:
        cv2.drawMarker(frame, (gfx, gfy), (0, 0, 220), cv2.MARKER_STAR, 14, 2)

    # --- Scale bar (5 m reference at bottom of panel) --------------------
    bar_len_px = int(5.0 * scale)               # 5 metres in pixels
    bar_y = panel_y + panel_size - 14
    bar_x0 = panel_x + 10
    bar_x1 = bar_x0 + bar_len_px
    cv2.line(frame, (bar_x0, bar_y), (bar_x1, bar_y), (180, 180, 180), 1)
    cv2.line(frame, (bar_x0, bar_y - 3), (bar_x0, bar_y + 3), (180, 180, 180), 1)
    cv2.line(frame, (bar_x1, bar_y - 3), (bar_x1, bar_y + 3), (180, 180, 180), 1)
    cv2.putText(frame, "5m", (bar_x0 + bar_len_px // 2 - 8, bar_y - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 180, 180), 1)

    # --- Info text (top-left of MAIN frame) ------------------------------
    state = nav_health.get("state", "?")
    wp_index = nav_health.get("wp_index", 0)
    wp_total = nav_health.get("wp_total", 0)
    occ_cells = int((grid > 40).sum())
    dist = math.hypot(rx - gx_goal, ry - gy_goal)

    lines = [
        ("Go1 Autonomous Navigation - LingTu", (255, 255, 255), 0.50, 1),
        ("t=%.1fs  pos=(%.1f, %.1f)  dist=%.1fm" % (elapsed, rx, ry, dist),
         (210, 210, 210), 0.42, 1),
        ("STATE: %s  wp=%d/%d  map=%d  scans=%d" % (
            state, wp_index, wp_total, occ_cells, scan_count),
         (180, 220, 180), 0.42, 1),
    ]
    for i, (txt, color, scale_f, thickness) in enumerate(lines):
        cv2.putText(frame, txt, (10, 22 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, scale_f, color, thickness,
                    cv2.LINE_AA)

    # --- SUCCESS banner ---------------------------------------------------
    if state == "SUCCESS":
        banner = "GOAL REACHED!"
        (tw, th), _ = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 1.3, 2)
        bx = (W - tw) // 2
        by = H // 2
        # Drop shadow
        cv2.putText(frame, banner, (bx + 2, by + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 80, 0), 3, cv2.LINE_AA)
        # Main text
        cv2.putText(frame, banner, (bx, by),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 2, cv2.LINE_AA)

    return frame
