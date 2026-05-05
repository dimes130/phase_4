from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

fl = robot.getDevice("green_front_left_motor")
fr = robot.getDevice("green_front_right_motor")
bl = robot.getDevice("green_back_left_motor")
br = robot.getDevice("green_back_right_motor")

for m in [fl, fr, bl, br]:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

camera = robot.getDevice("green_camera")
camera.enable(timestep)

WIDTH  = camera.getWidth()
HEIGHT = camera.getHeight()

left_arm  = robot.getDevice("green_left_arm_motor")
right_arm = robot.getDevice("green_right_arm_motor")
left_arm.setVelocity(0.5)
right_arm.setVelocity(0.5)

# Lidar is no longer used for possession detection.
# lidar = robot.getDevice("green_lidar")
# lidar.enable(timestep)

# ── tunable parameters ────────────────────────────────────────────────────────

START_DELAY          = 6.0
FORWARD_SPEED        = 5.0
SEARCH_SPEED         = 3.0
STEER_GAIN           = 0.05
MIN_PIXELS           = 20
CENTER_THRESHOLD     = 40

# Camera-based possession tuning
POSSESSION_PIXELS    = 3200
POSSESSION_CENTER    = 14
POSSESSION_FRAMES    = 45

FOLDED_POS   = -1.5708
DEPLOYED_POS =  0.0

# ── helpers ───────────────────────────────────────────────────────────────────

def set_speed(left, right):
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

# ── yellow ball detector: high R, high G, low B ──────────────────────────────

def get_centroid_and_count():
    """
    Detect yellow ball.
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

            # Yellow = high red, high green, low blue
            if r > 120 and g > 120 and b < 100:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None, 0

    error = (total_x / count) - (WIDTH / 2)
    return error, count

# ── green pocket detector: high G, low R, low B ──────────────────────────────

def get_pocket_centroid():
    """
    Detect green pocket.
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

            # Green pocket = high green, low red, low blue
            if g > 150 and r < 80 and b < 80:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None

    error = (total_x / count) - (WIDTH / 2)
    return error

# ── main loop ─────────────────────────────────────────────────────────────────

fold_arms()
wait(START_DELAY)

# Optional: drive forward briefly before scanning.
# Keep this only if it helps the green robot clear the starting area.
end_forward = robot.getTime() + 2.0
while robot.step(timestep) != -1:
    set_speed(FORWARD_SPEED, FORWARD_SPEED)
    if robot.getTime() >= end_forward:
        break

deployed = False
has_ball = False
possession_counter = 0

while robot.step(timestep) != -1:

    if not has_ball:
        error, count = get_centroid_and_count()

        if error is None:
            # Search clockwise for yellow ball
            set_speed(SEARCH_SPEED, -SEARCH_SPEED)
            possession_counter = 0

        else:
            # Steer toward yellow ball
            correction = STEER_GAIN * error
            left_speed = FORWARD_SPEED + correction
            right_speed = FORWARD_SPEED - correction
            set_speed(left_speed, right_speed)

            # Deploy arms once the ball is roughly centered
            if not deployed and abs(error) < CENTER_THRESHOLD:
                deploy_arms()
                deployed = True
                wait(0.5)

            # Camera-based possession check:
            # The ball must be centered and visually large for several frames.
            if deployed and abs(error) < POSSESSION_CENTER and count > POSSESSION_PIXELS:
                possession_counter += 1
            else:
                possession_counter = 0

            if possession_counter >= POSSESSION_FRAMES:
                has_ball = True
                stop()
                wait(0.2)

    else:
        # Now drive toward green pocket while holding/pushing the yellow ball
        error = get_pocket_centroid()

        if error is None:
            # Search for green pocket
            set_speed(SEARCH_SPEED, -SEARCH_SPEED)

        else:
            correction = STEER_GAIN * error
            left_speed = FORWARD_SPEED + correction
            right_speed = FORWARD_SPEED - correction
            set_speed(left_speed, right_speed)