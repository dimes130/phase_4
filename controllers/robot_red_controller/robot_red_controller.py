from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

fl = robot.getDevice("red_front_left_motor")
fr = robot.getDevice("red_front_right_motor")
bl = robot.getDevice("red_back_left_motor")
br = robot.getDevice("red_back_right_motor")

for m in [fl, fr, bl, br]:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

camera = robot.getDevice("red_camera")
camera.enable(timestep)

WIDTH  = camera.getWidth()
HEIGHT = camera.getHeight()

pocket_camera = robot.getDevice("red_pocket_camera")
pocket_camera.enable(timestep)

POCKET_WIDTH  = pocket_camera.getWidth()
POCKET_HEIGHT = pocket_camera.getHeight()

left_arm  = robot.getDevice("red_left_arm_motor")
right_arm = robot.getDevice("red_right_arm_motor")
left_arm.setVelocity(0.5)
right_arm.setVelocity(0.5)

# ── tunable parameters ────────────────────────────────────────────────────────

START_DELAY          = 16.0
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
POCKET_CENTER_LIMIT  = 18
BALL_KEEP_STEER_GAIN = 0.015
BALL_CENTER_LIMIT    = 16
BALL_RECENTER_LIMIT  = 55
MIN_CARRY_SPEED      = 1.0
MAX_CARRY_SPEED      = 6.0

# ── helpers ───────────────────────────────────────────────────────────────────

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
    left_arm.setPosition(FOLDED_POS)
    right_arm.setPosition(FOLDED_POS)

def deploy_arms():
    left_arm.setPosition(DEPLOYED_POS)
    right_arm.setPosition(DEPLOYED_POS)

def wait(seconds):
    stop()
    end = robot.getTime() + seconds
    while robot.step(timestep) != -1:
        if robot.getTime() >= end:
            break

def drive_forward_after_possession():
    end = robot.getTime() + POST_POSSESSION_DRIVE_TIME
    while robot.step(timestep) != -1:
        set_speed(FORWARD_SPEED, FORWARD_SPEED)
        if robot.getTime() >= end:
            break

    # Important: do NOT stop here.
    # The robot should smoothly transition into pocket-seeking.

# ── magenta ball detector: high R, high B, low G ─────────────────────────────

def get_centroid_and_count():
    """
    Detect magenta ball.
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

            # Magenta = high red, high blue, low green
            if r > 120 and b > 120 and g < 100:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None, 0

    error = (total_x / count) - (WIDTH / 2)
    return error, count

# ── red pocket detector: high R, low G, low B ────────────────────────────────

def get_pocket_centroid():
    """
    Detect the red pocket with the high mast camera.
    Returns (horizontal_error, pixel_count), or (None, 0) if not visible.
    """
    image = pocket_camera.getImage()
    if image is None:
        return None, 0

    total_x = 0
    count = 0

    for y in range(0, POCKET_HEIGHT, 4):
        for x in range(0, POCKET_WIDTH, 4):
            r = pocket_camera.imageGetRed(image, POCKET_WIDTH, x, y)
            g = pocket_camera.imageGetGreen(image, POCKET_WIDTH, x, y)
            b = pocket_camera.imageGetBlue(image, POCKET_WIDTH, x, y)

            # Red pocket = high red, low green, low blue
            if r > 150 and g < 80 and b < 80:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None, 0

    error = (total_x / count) - (POCKET_WIDTH / 2)
    return error, count

# ── main loop ─────────────────────────────────────────────────────────────────

fold_arms()
wait(START_DELAY)

deployed = False
has_ball = False
possession_counter = 0
full_view_counter = 0

while robot.step(timestep) != -1:

    if not has_ball:
        error, count = get_centroid_and_count()

        if error is None:
            # Search counterclockwise for magenta ball
            set_speed(-SEARCH_SPEED, SEARCH_SPEED)
            possession_counter = 0
            full_view_counter = 0

        else:
            # Steer toward magenta ball
            correction = STEER_GAIN * error
            left_speed = FORWARD_SPEED + correction
            right_speed = FORWARD_SPEED - correction
            set_speed(left_speed, right_speed)

            # Deploy arms once the ball is roughly centered
            if not deployed and abs(error) < CENTER_THRESHOLD:
                deploy_arms()
                deployed = True
                wait(0.5)

            # Normal possession check:
            # Ball must be centered and large for several frames.
            if deployed and abs(error) < POSSESSION_CENTER and count > POSSESSION_PIXELS:
                possession_counter += 1

            else:
                possession_counter = 0

            # Backup trigger:
            # If camera is almost completely filled by the ball for several frames,
            # confirm possession even if the normal counter resets later.
            if deployed and count >= FULL_VIEW_PIXELS:
                full_view_counter += 1

            else:
                full_view_counter = 0

            if possession_counter >= POSSESSION_FRAMES or full_view_counter >= FULL_VIEW_FRAMES:
                drive_forward_after_possession()
                has_ball = True
                continue

    else:
        # Now drive toward red pocket while holding/pushing the magenta ball.
        # Never rotate in place here — always keep forward momentum.
        pocket_error, pocket_pixels = get_pocket_centroid()
        ball_error, ball_pixels = get_centroid_and_count()

        pocket_correction = 0.0

        if pocket_error is None:
            # Curved search instead of spinning in place.
            # Both wheel sides stay positive so the robot keeps carrying the ball forward.
            pocket_correction = POCKET_TURN_BIAS
        else:
            # Use the high pocket camera to keep the pocket centered while carrying the ball.
            if abs(pocket_error) >= POCKET_CENTER_LIMIT:
                pocket_correction = POCKET_STEER_GAIN * pocket_error

        ball_correction = 0.0
        if ball_error is not None and abs(ball_error) >= BALL_CENTER_LIMIT:
            ball_correction = BALL_KEEP_STEER_GAIN * ball_error

        if ball_error is not None and abs(ball_error) >= BALL_RECENTER_LIMIT:
            correction = ball_correction
        else:
            correction = pocket_correction + ball_correction

        left_speed = POCKET_FORWARD_SPEED + correction
        right_speed = POCKET_FORWARD_SPEED - correction

        # Keep both sides moving forward so it does not pivot in place.
        left_speed = max(MIN_CARRY_SPEED, min(MAX_CARRY_SPEED, left_speed))
        right_speed = max(MIN_CARRY_SPEED, min(MAX_CARRY_SPEED, right_speed))

        set_speed(left_speed, right_speed)
