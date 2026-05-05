"""Ball Pusher Controller for Pioneer 3-DX — Webots Billiards Competition.

customData format: "color pocket_x pocket_y delay_seconds"

Strategy:
  1. Exit start box → center → behind ball (via waypoints avoiding obstacles)
  2. Approach ball slowly using go_to (aims naturally at ball)
  3. Push ball to pocket using continuous go_to(pocket) — self-correcting
  4. If push fails (timeout), back up and retry

State machine: WAIT → NAV → APPROACH → PUSH → RETREAT → DONE
                                          ↑              |
                                          └── RETRY ─────┘
"""

from controller import Robot
import math
import sys

# ── Initialize ──────────────────────────────────────────────────────
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep()) or 16
NAME = robot.getName()

params = robot.getCustomData().split()
if len(params) < 4:
    print(f"[{NAME}] ERROR: need 4 params in customData, got {len(params)}")
    sys.exit(1)

TARGET_COLOR = params[0]
POCKET_X, POCKET_Y = float(params[1]), float(params[2])
DELAY = float(params[3])

# All ball starting positions
ALL_BALLS = {
    'yellow':  (-5.05, 0.0),
    'purple':  (-3.93, 3.55),
    'magenta': (-2.63, 0.55),
    'cyan':    (-0.48, 3.24),
}
ball_x, ball_y = ALL_BALLS[TARGET_COLOR]

print(f"[{NAME}] target={TARGET_COLOR} ball=({ball_x},{ball_y}) pocket=({POCKET_X},{POCKET_Y}) delay={DELAY}s")

# ── Devices ─────────────────────────────────────────────────────────
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
gps = robot.getDevice('gps')
compass = robot.getDevice('compass')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

gps.enable(TIME_STEP)
compass.enable(TIME_STEP)

# ── Constants ───────────────────────────────────────────────────────
MAX_SPEED = 6.0
BEHIND_DIST = 2.0
PUSH_OVERSHOOT = 1.5

# Arena safe bounds (walls + margin)
SAFE_X_MIN, SAFE_X_MAX = -6.3, 2.3
SAFE_Y_MIN, SAFE_Y_MAX = -2.8, 5.8
ARENA_CENTER = (-2.0, 1.5)

# ── Helpers ─────────────────────────────────────────────────────────
def get_pos():
    v = gps.getValues()
    return v[0], v[1]

def get_heading():
    c = compass.getValues()
    return math.atan2(c[0], c[1])

def wrap(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def dist(ax, ay, bx, by):
    return math.sqrt((bx - ax)**2 + (by - ay)**2)

def dist_to(tx, ty):
    x, y = get_pos()
    return dist(x, y, tx, ty)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def set_vel(l, r):
    left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, l)))
    right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, r)))

def stop():
    set_vel(0, 0)

def go_to(tx, ty, speed=MAX_SPEED, arrive_dist=0.3):
    """Drive toward (tx,ty) using GPS. Returns True when arrived."""
    x, y = get_pos()
    dx, dy = tx - x, ty - y
    d = math.sqrt(dx*dx + dy*dy)
    if d < arrive_dist:
        stop()
        return True
    bearing = math.atan2(dy, dx)
    err = wrap(bearing - get_heading())
    fwd = speed * min(1.0, d / 1.0)
    turn = 5.0 * err
    set_vel(fwd - turn, fwd + turn)
    return False


# ── Obstacle detection ──────────────────────────────────────────────
def point_to_segment_dist(px, py, ax, ay, bx, by):
    """Distance from point (px,py) to line segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    len_sq = dx*dx + dy*dy
    if len_sq < 0.001:
        return dist(px, py, ax, ay)
    t = max(0, min(1, ((px - ax)*dx + (py - ay)*dy) / len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return dist(px, py, proj_x, proj_y)


def find_obstacles_in_path(bx, by, px, py):
    """Return list of (name, x, y) for balls that are within 1.0m of the push line."""
    obstacles = []
    for name, (ox, oy) in ALL_BALLS.items():
        if name == TARGET_COLOR:
            continue  # skip our own ball
        d = point_to_segment_dist(ox, oy, bx, by, px, py)
        if d < 1.0:
            obstacles.append((name, ox, oy))
    return obstacles


# ── Path planning ───────────────────────────────────────────────────
def compute_behind_point(bx, by, px, py, behind_dist=BEHIND_DIST):
    """Compute the point behind the ball, opposite from pocket."""
    dx = px - bx
    dy = py - by
    d = math.sqrt(dx*dx + dy*dy)
    if d < 0.01:
        return bx, by
    ux, uy = dx / d, dy / d
    rx = clamp(bx - ux * behind_dist, SAFE_X_MIN, SAFE_X_MAX)
    ry = clamp(by - uy * behind_dist, SAFE_Y_MIN, SAFE_Y_MAX)
    return rx, ry


def build_waypoints(bx, by, px, py):
    """Build navigation waypoints: exit → center → behind ball.
    If obstacles are in the push path, add waypoints to avoid them."""
    behind_x, behind_y = compute_behind_point(bx, by, px, py)

    obstacles = find_obstacles_in_path(bx, by, px, py)

    wps = [
        (0.0, 0.5),      # exit start box
        ARENA_CENTER,     # center of arena
    ]

    if obstacles:
        names = [o[0] for o in obstacles]
        print(f"[{NAME}] Obstacles in push path: {names}")
        # Route the behind-point wider to avoid pushing into obstacles
        # Offset perpendicular to the push direction
        dx = px - bx
        dy = py - by
        d = math.sqrt(dx*dx + dy*dy)
        # Perpendicular direction (try left first, then right)
        perp_x, perp_y = -dy / d, dx / d

        # Try offsetting the ball push line by 1.5m perpendicular
        for sign in [1, -1]:
            offset_bx = bx + sign * perp_x * 1.5
            offset_by = by + sign * perp_y * 1.5
            # Check this offset is in safe bounds and has no obstacles
            if (SAFE_X_MIN < offset_bx < SAFE_X_MAX and
                SAFE_Y_MIN < offset_by < SAFE_Y_MAX):
                new_obs = find_obstacles_in_path(offset_bx, offset_by, px, py)
                if not new_obs:
                    # Push ball sideways first to this clear position,
                    # then push to pocket
                    # We need: go behind ball relative to offset point, push to offset,
                    # then go behind offset relative to pocket, push to pocket
                    mid_behind_x, mid_behind_y = compute_behind_point(
                        bx, by, offset_bx, offset_by, behind_dist=1.5)
                    wps.append((mid_behind_x, mid_behind_y))
                    print(f"[{NAME}] Rerouting via offset ({offset_bx:.1f},{offset_by:.1f})")
                    return wps, offset_bx, offset_by, True
        # If no clear offset found, just push straight and hope for the best
        print(f"[{NAME}] No clear detour found, pushing straight")

    wps.append((behind_x, behind_y))
    return wps, bx, by, False


# ── Build initial plan ──────────────────────────────────────────────
waypoints, target_ball_x, target_ball_y, has_detour = build_waypoints(
    ball_x, ball_y, POCKET_X, POCKET_Y)

# Push target: past the pocket
dx_bp = POCKET_X - ball_x
dy_bp = POCKET_Y - ball_y
d_bp = math.sqrt(dx_bp**2 + dy_bp**2)
ux_bp, uy_bp = dx_bp / d_bp, dy_bp / d_bp

print(f"[{NAME}] waypoints: {[(f'{x:.1f}',f'{y:.1f}') for x,y in waypoints]}")
if has_detour:
    print(f"[{NAME}] Will push ball sideways first to avoid obstacle")

# ── State Machine ───────────────────────────────────────────────────
state = 'WAIT'
clock = 0.0
wp_index = 0
push_start = 0.0
retry_count = 0
MAX_RETRIES = 2

# Detour state tracking
detour_phase = 0  # 0 = pushing to detour point, 1 = re-aligned for pocket

while robot.step(TIME_STEP) != -1:
    dt = TIME_STEP / 1000.0
    clock += dt

    # ── WAIT ────────────────────────────────────────────────────
    if state == 'WAIT':
        stop()
        if clock >= DELAY:
            print(f"[{NAME}] WAIT -> NAV  t={clock:.1f}")
            state = 'NAV'
            wp_index = 0

    # ── NAV: follow waypoints ───────────────────────────────────
    elif state == 'NAV':
        tx, ty = waypoints[wp_index]
        if go_to(tx, ty, speed=MAX_SPEED, arrive_dist=0.5):
            print(f"[{NAME}] NAV: wp {wp_index} reached ({tx:.1f},{ty:.1f})")
            wp_index += 1
            if wp_index >= len(waypoints):
                print(f"[{NAME}] NAV -> APPROACH")
                state = 'APPROACH'

    # ── APPROACH: slowly drive toward the ball ──────────────────
    elif state == 'APPROACH':
        if go_to(target_ball_x, target_ball_y, speed=MAX_SPEED * 0.35, arrive_dist=0.55):
            print(f"[{NAME}] APPROACH -> PUSH")
            state = 'PUSH'
            push_start = clock

    # ── PUSH: continuously aim at pocket (self-correcting) ──────
    elif state == 'PUSH':
        if has_detour and detour_phase == 0:
            # Phase 1: push ball to the detour point (sideways)
            if go_to(target_ball_x, target_ball_y, speed=MAX_SPEED * 0.4, arrive_dist=0.8):
                print(f"[{NAME}] Detour push complete, re-aligning for pocket")
                detour_phase = 1
                # Now re-plan: ball is at the detour point, push to pocket
                new_behind_x, new_behind_y = compute_behind_point(
                    target_ball_x, target_ball_y, POCKET_X, POCKET_Y)
                waypoints = [(new_behind_x, new_behind_y)]
                wp_index = 0
                target_ball_x, target_ball_y = target_ball_x, target_ball_y
                has_detour = False
                state = 'RETREAT_TEMP'
                continue
        else:
            # Normal push: continuously aim at pocket
            go_to(POCKET_X, POCKET_Y, speed=MAX_SPEED * 0.45, arrive_dist=0.8)

            if dist_to(POCKET_X, POCKET_Y) < 1.0:
                print(f"[{NAME}] PUSH -> RETREAT  t={clock:.1f}")
                state = 'RETREAT'

        # Timeout: if pushing too long, retry
        if clock - push_start > 40.0:
            if retry_count < MAX_RETRIES:
                print(f"[{NAME}] PUSH timeout -> RETRY (attempt {retry_count + 1})")
                state = 'RETRY'
            else:
                print(f"[{NAME}] PUSH timeout -> RETREAT (max retries)")
                state = 'RETREAT'

    # ── RETRY: back up, re-estimate ball position, try again ────
    elif state == 'RETRY':
        # Back up first
        set_vel(-3.0, -3.0)
        if dist_to(POCKET_X, POCKET_Y) > 3.0:
            stop()
            retry_count += 1
            # Estimate where ball is now: somewhere between start and pocket
            # along the push line. Use our current position as a rough guide —
            # ball is likely slightly ahead of us.
            rx, ry = get_pos()
            # The ball is probably near our position, biased toward pocket
            est_bx = rx + ux_bp * 0.5
            est_by = ry + uy_bp * 0.5

            # Re-plan from estimated ball position
            new_behind_x, new_behind_y = compute_behind_point(
                est_bx, est_by, POCKET_X, POCKET_Y, behind_dist=1.5)

            waypoints = [
                ARENA_CENTER,
                (new_behind_x, new_behind_y),
            ]
            target_ball_x, target_ball_y = est_bx, est_by
            wp_index = 0
            print(f"[{NAME}] RETRY: est ball=({est_bx:.1f},{est_by:.1f}), re-navigating")
            state = 'NAV'

    # ── RETREAT_TEMP: back up after detour push, then re-navigate ──
    elif state == 'RETREAT_TEMP':
        set_vel(-3.0, -3.0)
        if dist_to(target_ball_x, target_ball_y) > 2.0:
            stop()
            state = 'NAV'

    # ── RETREAT: back away from pocket ──────────────────────────
    elif state == 'RETREAT':
        set_vel(-3.0, -3.0)
        if dist_to(POCKET_X, POCKET_Y) > 2.5:
            stop()
            print(f"[{NAME}] RETREAT -> DONE  t={clock:.1f}")
            state = 'DONE'

    # ── DONE ────────────────────────────────────────────────────
    elif state == 'DONE':
        stop()
