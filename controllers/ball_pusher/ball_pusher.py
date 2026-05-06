"""Ball Pusher Controller for Pioneer 3-DX — Webots Billiards Competition.

customData format: "color pocket_x pocket_y delay_seconds"

Strategy:
  First attempt: use hardcoded ball positions + GPS nav (reliable)
  On retry: use camera to search for ball, center it, drive toward it,
            then push to pocket with go_to(pocket)

State machine: WAIT → NAV → APPROACH → PUSH → RETREAT → DONE
                                         ↑              |
                                         └── SEARCH ────┘
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

# Known ball starting positions (from world file)
# Used as initial estimates — updated during retries
BALL_START = {
    'yellow':  (-5.05, 0.0),
    'magenta': (-2.63, 0.55),
    'cyan':    (-0.48, 3.24),
}

# All ball positions (for obstacle awareness)
ALL_BALLS = {
    'yellow':  (-5.05, 0.0),
    'purple':  (-3.93, 3.55),
    'magenta': (-2.63, 0.55),
    'cyan':    (-0.48, 3.24),
}

# Current estimated ball position (starts at known position, updated on retry)
ball_x, ball_y = BALL_START[TARGET_COLOR]

print(f"[{NAME}] target={TARGET_COLOR} ball=({ball_x},{ball_y}) pocket=({POCKET_X},{POCKET_Y}) delay={DELAY}s")

# Ball color RGB values (0-1 scale)
COLOR_TABLE = {
    'yellow':  (1.0, 1.0, 0.0),
    'magenta': (1.0, 0.0, 1.0),
    'cyan':    (0.0, 1.0, 1.0),
    'purple':  (0.5, 0.0, 1.0),
}

# ── Devices ─────────────────────────────────────────────────────────
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
gps = robot.getDevice('gps')
compass = robot.getDevice('compass')
camera = robot.getDevice('camera')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

gps.enable(TIME_STEP)
compass.enable(TIME_STEP)
camera.enable(TIME_STEP)

CAM_W = camera.getWidth()   # 160
CAM_H = camera.getHeight()  # 120

# ── Constants ───────────────────────────────────────────────────────
MAX_SPEED = 6.0
BEHIND_DIST = 2.0

# Arena safe bounds
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

def ddist(ax, ay, bx, by):
    return math.sqrt((bx - ax)**2 + (by - ay)**2)

def dist_to(tx, ty):
    x, y = get_pos()
    return ddist(x, y, tx, ty)

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


# ── Camera Helpers ──────────────────────────────────────────────────
def see_ball(color_name):
    """Look for color_name in camera. Returns (found, offset).
    offset: -1.0 (left) to +1.0 (right), 0 = centered.
    """
    img = camera.getImage()
    if not img:
        return False, 0.0

    tr, tg, tb = COLOR_TABLE[color_name]
    cx_sum = 0
    count = 0
    step = 2

    for px in range(0, CAM_W, step):
        for py in range(CAM_H // 5, 4 * CAM_H // 5, step):
            r = camera.imageGetRed(img, CAM_W, px, py) / 255.0
            g = camera.imageGetGreen(img, CAM_W, px, py) / 255.0
            b = camera.imageGetBlue(img, CAM_W, px, py) / 255.0
            if abs(r - tr) < 0.3 and abs(g - tg) < 0.3 and abs(b - tb) < 0.3:
                cx_sum += px
                count += 1

    if count > 5:
        offset = (cx_sum / count / CAM_W - 0.5) * 2.0
        return True, offset
    return False, 0.0


def steer_toward_ball(color_name, speed=3.0):
    """Steer to center the ball in the camera and drive forward.
    Returns (found, centered) — centered means ball is roughly in front."""
    found, offset = see_ball(color_name)
    if not found:
        return False, False
    # Proportional steering to center the ball
    steer = offset * 4.0
    set_vel(speed - steer, speed + steer)
    return True, abs(offset) < 0.15


# ── Path Planning ───────────────────────────────────────────────────
def compute_behind_point(bx, by, px, py, behind_dist=BEHIND_DIST):
    """Point behind ball, opposite from pocket, clamped to safe area."""
    dx = px - bx
    dy = py - by
    d = math.sqrt(dx*dx + dy*dy)
    if d < 0.01:
        return bx, by
    ux, uy = dx / d, dy / d
    rx = clamp(bx - ux * behind_dist, SAFE_X_MIN, SAFE_X_MAX)
    ry = clamp(by - uy * behind_dist, SAFE_Y_MIN, SAFE_Y_MAX)
    return rx, ry


def point_to_segment_dist(px, py, ax, ay, bx, by):
    """Distance from point (px,py) to line segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    len_sq = dx*dx + dy*dy
    if len_sq < 0.001:
        return ddist(px, py, ax, ay)
    t = max(0, min(1, ((px - ax)*dx + (py - ay)*dy) / len_sq))
    proj_x = ax + t * dx
    proj_y = ay + t * dy
    return ddist(px, py, proj_x, proj_y)


def check_obstacles(bx, by, px, py):
    """Check if any other ball is in the push path. Returns list of names."""
    obstacles = []
    for name, (ox, oy) in ALL_BALLS.items():
        if name == TARGET_COLOR:
            continue
        d = point_to_segment_dist(ox, oy, bx, by, px, py)
        if d < 1.0:
            obstacles.append(name)
    return obstacles


def check_leg_obstacles(ax, ay, bx, by, margin=1.2):
    """Check if any non-target ball is within margin of the leg from (ax,ay) to (bx,by).
    Returns list of (name, ox, oy) for balls too close."""
    hits = []
    for name, (ox, oy) in ALL_BALLS.items():
        if name == TARGET_COLOR:
            continue
        d = point_to_segment_dist(ox, oy, ax, ay, bx, by)
        if d < margin:
            hits.append((name, ox, oy))
    return hits


def detour_around(ax, ay, bx, by, ox, oy, clearance=1.8):
    """Compute a detour waypoint to avoid obstacle at (ox,oy) on the path (ax,ay)->(bx,by).
    Returns a point offset perpendicular to the path, on whichever side is safer."""
    dx, dy = bx - ax, by - ay
    d = math.sqrt(dx*dx + dy*dy)
    if d < 0.01:
        return ax, ay
    # Perpendicular directions
    perp_x, perp_y = -dy / d, dx / d
    # Midpoint of the leg (roughly where the obstacle is along the path)
    mx, my = (ax + bx) / 2, (ay + by) / 2
    # Try both sides, pick the one that stays more in-bounds
    for sign in [1, -1]:
        wx = mx + sign * perp_x * clearance
        wy = my + sign * perp_y * clearance
        if SAFE_X_MIN < wx < SAFE_X_MAX and SAFE_Y_MIN < wy < SAFE_Y_MAX:
            # Verify the detour doesn't pass near the obstacle either
            d1 = point_to_segment_dist(ox, oy, ax, ay, wx, wy)
            d2 = point_to_segment_dist(ox, oy, wx, wy, bx, by)
            if d1 > 1.0 and d2 > 1.0:
                return wx, wy
    # Fallback: just offset from obstacle position directly
    return ox + perp_x * clearance, oy + perp_y * clearance


def build_waypoints(bx, by):
    """Build waypoints from start box to behind the ball, avoiding other balls."""
    behind_x, behind_y = compute_behind_point(bx, by, POCKET_X, POCKET_Y)

    obstacles = check_obstacles(bx, by, POCKET_X, POCKET_Y)
    if obstacles:
        print(f"[{NAME}] WARNING: {obstacles} in push path")

    # Base route
    raw_wps = [
        (0.0, 0.5),       # exit start box
        ARENA_CENTER,      # safe center point
        (behind_x, behind_y),
    ]

    # Check each leg for obstacle balls and insert detours
    final_wps = [raw_wps[0]]
    for i in range(len(raw_wps) - 1):
        ax, ay = raw_wps[i]
        bx_wp, by_wp = raw_wps[i + 1]
        hits = check_leg_obstacles(ax, ay, bx_wp, by_wp)
        if hits:
            for obs_name, ox, oy in hits:
                wx, wy = detour_around(ax, ay, bx_wp, by_wp, ox, oy)
                print(f"[{NAME}] Detouring around {obs_name} at ({ox:.1f},{oy:.1f}) via ({wx:.1f},{wy:.1f})")
                final_wps.append((wx, wy))
        final_wps.append((bx_wp, by_wp))

    return final_wps


# ── Build initial plan ──────────────────────────────────────────────
waypoints = build_waypoints(ball_x, ball_y)
print(f"[{NAME}] waypoints: {[(f'{x:.1f}',f'{y:.1f}') for x,y in waypoints]}")

# ── State Machine ───────────────────────────────────────────────────
state = 'WAIT'
clock = 0.0
wp_index = 0
push_start = 0.0
retry_count = 0
MAX_RETRIES = 3
search_spin_time = 0.0
search_nav_index = 0
# Search positions to drive to when looking for ball with camera
SEARCH_SPOTS = [(-2.0, 1.5), (-4.5, 1.5), (-1.0, 3.5), (-4.5, 4.0), (0.5, 1.0)]
searching_to_spot = False  # True = driving to search spot, False = spinning

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

    # ── NAV: follow waypoints to get behind ball ────────────────
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
        if go_to(ball_x, ball_y, speed=MAX_SPEED * 0.35, arrive_dist=0.55):
            print(f"[{NAME}] APPROACH -> PUSH")
            state = 'PUSH'
            push_start = clock

    # ── PUSH: continuously aim at pocket (self-correcting) ──────
    elif state == 'PUSH':
        d_pocket = dist_to(POCKET_X, POCKET_Y)
        # Slow down as we approach the pocket — gentle entry prevents bouncing
        if d_pocket < 1.5:
            push_speed = MAX_SPEED * 0.25
        elif d_pocket < 3.0:
            push_speed = MAX_SPEED * 0.35
        else:
            push_speed = MAX_SPEED * 0.45
        go_to(POCKET_X, POCKET_Y, speed=push_speed, arrive_dist=0.3)

        if d_pocket < 0.5:
            print(f"[{NAME}] PUSH -> RETREAT  t={clock:.1f}")
            state = 'RETREAT'

        # Timeout — ball may have escaped, use camera to re-find it
        if clock - push_start > 40.0:
            if retry_count < MAX_RETRIES:
                retry_count += 1
                print(f"[{NAME}] PUSH timeout -> BACKUP (retry {retry_count})")
                state = 'BACKUP'
            else:
                print(f"[{NAME}] PUSH timeout -> RETREAT (max retries)")
                state = 'RETREAT'

    # ── BACKUP: reverse away before searching ───────────────────
    elif state == 'BACKUP':
        set_vel(-3.0, -3.0)
        if dist_to(POCKET_X, POCKET_Y) > 3.5:
            stop()
            print(f"[{NAME}] BACKUP -> SEARCH")
            state = 'SEARCH'
            search_spin_time = 0.0
            search_nav_index = 0
            searching_to_spot = False

    # ── SEARCH: spin + drive to spots to find ball with camera ──
    elif state == 'SEARCH':
        if searching_to_spot:
            # Drive to next search spot
            sx, sy = SEARCH_SPOTS[search_nav_index]
            if go_to(sx, sy, speed=MAX_SPEED, arrive_dist=0.5):
                searching_to_spot = False
                search_spin_time = 0.0
        else:
            # Spin in place and check camera
            search_spin_time += dt
            found, offset = see_ball(TARGET_COLOR)

            if found and abs(offset) < 0.4:
                print(f"[{NAME}] SEARCH: spotted {TARGET_COLOR} (offset={offset:.2f})")
                state = 'CHASE'
                continue

            # Slow spin
            set_vel(-1.5, 1.5)

            # After one full rotation (~6s), move to next search spot
            if search_spin_time > 6.0:
                search_nav_index = (search_nav_index + 1) % len(SEARCH_SPOTS)
                searching_to_spot = True
                print(f"[{NAME}] SEARCH: moving to spot {search_nav_index}")

    # ── CHASE: camera-track the ball, drive toward it ───────────
    elif state == 'CHASE':
        found, centered = steer_toward_ball(TARGET_COLOR, speed=MAX_SPEED * 0.35)

        if not found:
            # Lost it — go back to searching
            print(f"[{NAME}] CHASE: lost ball, back to SEARCH")
            state = 'SEARCH'
            search_spin_time = 0.0
            continue

        # Check if we're close to the ball — use distance sensor proxy:
        # if the ball blob is large in the camera, we're close enough
        _, ball_offset = see_ball(TARGET_COLOR)
        # Simple heuristic: once we've been chasing for a bit and are
        # driving straight at it, switch to push
        # We know we're close when the ball is centered and we've driven a bit
        rx, ry = get_pos()
        # We can check: are we roughly between the ball's last known area and pocket?
        # Just push toward pocket once ball is centered
        if centered:
            print(f"[{NAME}] CHASE -> PUSH (ball centered, pushing to pocket)")
            state = 'PUSH'
            push_start = clock

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
