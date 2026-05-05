from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

fl = robot.getDevice("blue_front_left_motor")
fr = robot.getDevice("blue_front_right_motor")
bl = robot.getDevice("blue_back_left_motor")
br = robot.getDevice("blue_back_right_motor")

for m in [fl, fr, bl, br]:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

camera = robot.getDevice("blue_camera")
camera.enable(timestep)

WIDTH  = camera.getWidth()
HEIGHT = camera.getHeight()

left_arm  = robot.getDevice("blue_left_arm_motor")
right_arm = robot.getDevice("blue_right_arm_motor")
left_arm.setVelocity(0.5)
right_arm.setVelocity(0.5)

# ── tunable parameters ────────────────────────────────────────────────────────

START_DELAY          = 0.0
FORWARD_SPEED        = 5.0
SEARCH_SPEED         = 3.0
STEER_GAIN           = 0.05
MIN_PIXELS           = 20
CENTER_THRESHOLD     = 40

# Green-style possession, but low enough to actually switch
POSSESSION_PIXELS    = 3200
POSSESSION_CENTER    = 14
POSSESSION_FRAMES    = 20

# Backup possession trigger:
# if the ball basically fills the camera for several frames, accept possession
FULL_VIEW_PIXELS     = 4700
FULL_VIEW_FRAMES     = 8

FOLDED_POS   = -1.5708
DEPLOYED_POS =  0.0

MAX_MOTOR_SPEED = 10.0

# Drive forward briefly after possession so the ball fully seats in the arms
POST_POSSESSION_DRIVE_TIME = 0

# Pocket-seeking movement: curve forward, do not spin in place
POCKET_FORWARD_SPEED = 3.0
POCKET_TURN_BIAS     = 1.2
POCKET_STEER_GAIN    = 0.025
MIN_CARRY_SPEED      = 1.0
MAX_CARRY_SPEED      = 6.0

# Debug controls
DEBUG = True
DEBUG_INTERVAL = 0.5

# ── helpers ───────────────────────────────────────────────────────────────────

def debug(msg):
    if DEBUG:
        print(f"[BLUE t={robot.getTime():.2f}] {msg}")

def clamp(value, low=-MAX_MOTOR_SPEED, high=MAX_MOTOR_SPEED):
    return max(low, min(high, value))

def set_speed(left, right):
    left = clamp(left)
    right = clamp(right)

    fl.setVelocity(left)
    bl.setVelocity(left)
    fr.setVelocity(right)
    br.setVelocity(right)

def stop():
    set_speed(0.0, 0.0)

def fold_arms():
    debug("Folding arms")
    left_arm.setPosition(FOLDED_POS)
    right_arm.setPosition(FOLDED_POS)

def deploy_arms():
    debug("Deploying arms")
    left_arm.setPosition(DEPLOYED_POS)
    right_arm.setPosition(DEPLOYED_POS)

def wait(seconds):
    stop()
    end = robot.getTime() + seconds
    while robot.step(timestep) != -1:
        if robot.getTime() >= end:
            break

def drive_forward_after_possession():
    debug(f"Driving forward {POST_POSSESSION_DRIVE_TIME}s after possession")
    end = robot.getTime() + POST_POSSESSION_DRIVE_TIME
    while robot.step(timestep) != -1:
        set_speed(FORWARD_SPEED, FORWARD_SPEED)
        if robot.getTime() >= end:
            break

    # Important: do NOT stop here.
    # The robot should smoothly transition into pocket-seeking.
    debug("Finished post-possession drive")

# ── cyan ball detector: high G, high B, low R ────────────────────────────────

def get_centroid_and_count():
    """
    Detect cyan ball.
    Returns (horizontal_error, pixel_count).
    horizontal_error < 0 means ball is left of center.
    horizontal_error > 0 means ball is right of center.
    """
    image = camera.getImage()
    if image is None:
        return None, 0

    total_x = 0
    count = 0

    for y in range(0, HEIGHT, 4):
        for x in range(0, WIDTH, 4):
            r = camera.imageGetRed(image, WIDTH, x, y)
            g = camera.imageGetGreen(image, WIDTH, x, y)
            b = camera.imageGetBlue(image, WIDTH, x, y)

            # Cyan = high green, high blue, low red
            if g > 120 and b > 120 and r < 100:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None, 0

    error = (total_x / count) - (WIDTH / 2)
    return error, count

# ── blue pocket detector: high B, low R, low G ───────────────────────────────

def get_pocket_centroid():
    """
    Detect blue pocket.
    Returns horizontal_error or None.
    """
    image = camera.getImage()
    if image is None:
        return None

    total_x = 0
    count = 0

    for y in range(0, HEIGHT, 4):
        for x in range(0, WIDTH, 4):
            r = camera.imageGetRed(image, WIDTH, x, y)
            g = camera.imageGetGreen(image, WIDTH, x, y)
            b = camera.imageGetBlue(image, WIDTH, x, y)

            # Blue pocket = high blue, low red, low green
            if b > 150 and r < 80 and g < 80:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None

    error = (total_x / count) - (WIDTH / 2)
    return error

# ── main loop ─────────────────────────────────────────────────────────────────

fold_arms()
wait(START_DELAY)

debug("Starting mission")

deployed = False
has_ball = False
possession_counter = 0
full_view_counter = 0
last_debug_time = 0.0
last_counter_print = -1
last_full_view_print = -1

while robot.step(timestep) != -1:

    now = robot.getTime()

    if not has_ball:
        error, count = get_centroid_and_count()

        if now - last_debug_time >= DEBUG_INTERVAL:
            debug(
                f"BALL MODE | error={error} count={count} deployed={deployed} "
                f"possession_counter={possession_counter} full_view_counter={full_view_counter}"
            )
            last_debug_time = now

        if error is None:
            # Search clockwise for cyan ball
            set_speed(SEARCH_SPEED, -SEARCH_SPEED)
            possession_counter = 0
            full_view_counter = 0

        else:
            # Steer toward cyan ball
            correction = STEER_GAIN * error
            left_speed = FORWARD_SPEED + correction
            right_speed = FORWARD_SPEED - correction
            set_speed(left_speed, right_speed)

            # Deploy arms once the ball is roughly centered
            if not deployed and abs(error) < CENTER_THRESHOLD:
                debug(f"BALL CENTERED | deploying arms | error={error:.1f} count={count}")
                deploy_arms()
                deployed = True
                wait(0.5)

            # Normal possession check:
            # Ball must be centered and large for several frames.
            if deployed and abs(error) < POSSESSION_CENTER and count > POSSESSION_PIXELS:
                possession_counter += 1

                if possession_counter != last_counter_print and possession_counter % 5 == 0:
                    debug(
                        f"POSSESSION BUILDING | counter={possession_counter}/{POSSESSION_FRAMES} "
                        f"error={error:.1f} count={count}"
                    )
                    last_counter_print = possession_counter

            else:
                if possession_counter > 0:
                    debug(
                        f"POSSESSION RESET | counter was {possession_counter} | "
                        f"error={error:.1f} count={count}"
                    )
                possession_counter = 0

            # Backup trigger:
            # If camera is almost completely filled by the ball for several frames,
            # confirm possession even if the normal counter resets later.
            if deployed and count >= FULL_VIEW_PIXELS:
                full_view_counter += 1

                if full_view_counter != last_full_view_print and full_view_counter % 2 == 0:
                    debug(
                        f"FULL VIEW BUILDING | counter={full_view_counter}/{FULL_VIEW_FRAMES} "
                        f"count={count} error={error:.1f}"
                    )
                    last_full_view_print = full_view_counter

            else:
                if full_view_counter > 0:
                    debug(
                        f"FULL VIEW RESET | counter was {full_view_counter} | "
                        f"count={count} error={error:.1f}"
                    )
                full_view_counter = 0

            if possession_counter >= POSSESSION_FRAMES or full_view_counter >= FULL_VIEW_FRAMES:
                debug("POSSESSION CONFIRMED")
                drive_forward_after_possession()
                has_ball = True
                debug("HAS_BALL = TRUE | SWITCHING TO POCKET MODE")
                continue

    else:
        # Now drive toward blue pocket while holding/pushing the cyan ball.
        # Never rotate in place here — always keep forward momentum.
        error = get_pocket_centroid()

        if now - last_debug_time >= DEBUG_INTERVAL:
            debug(f"POCKET MODE | pocket_error={error}")
            last_debug_time = now

        if error is None:
            # Curved search instead of spinning in place.
            # Both wheel sides stay positive so the robot keeps carrying the ball forward.
            left_speed = POCKET_FORWARD_SPEED + POCKET_TURN_BIAS
            right_speed = POCKET_FORWARD_SPEED - POCKET_TURN_BIAS
            set_speed(left_speed, right_speed)

        else:
            # Curve toward the pocket while moving forward.
            correction = POCKET_STEER_GAIN * error

            left_speed = POCKET_FORWARD_SPEED + correction
            right_speed = POCKET_FORWARD_SPEED - correction

            # Keep both sides moving forward so it does not pivot in place.
            left_speed = max(MIN_CARRY_SPEED, min(MAX_CARRY_SPEED, left_speed))
            right_speed = max(MIN_CARRY_SPEED, min(MAX_CARRY_SPEED, right_speed))

            set_speed(left_speed, right_speed)