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

left_arm  = robot.getDevice("red_left_arm_motor")
right_arm = robot.getDevice("red_right_arm_motor")
left_arm.setVelocity(0.5)
right_arm.setVelocity(0.5)

# Lidar is no longer used for possession detection.
# lidar = robot.getDevice("red_lidar")
# lidar.enable(timestep)

# ── tunable parameters ────────────────────────────────────────────────────────

START_DELAY          = 16.0
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
    Detect red pocket.
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

            # Red pocket = high red, low green, low blue
            if r > 150 and g < 80 and b < 80:
                total_x += x
                count += 1

    if count < MIN_PIXELS:
        return None

    error = (total_x / count) - (WIDTH / 2)
    return error

# ── main loop ─────────────────────────────────────────────────────────────────

fold_arms()
wait(START_DELAY)

deployed = False
has_ball = False
possession_counter = 0

while robot.step(timestep) != -1:

    if not has_ball:
        error, count = get_centroid_and_count()

        if error is None:
            # Search counterclockwise for magenta ball
            set_speed(-SEARCH_SPEED, SEARCH_SPEED)
            possession_counter = 0

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
        # Now drive toward red pocket while holding/pushing the magenta ball
        error = get_pocket_centroid()

        if error is None:
            # Search for red pocket
            set_speed(-SEARCH_SPEED, SEARCH_SPEED)

        else:
            correction = STEER_GAIN * error
            left_speed = FORWARD_SPEED + correction
            right_speed = FORWARD_SPEED - correction
            set_speed(left_speed, right_speed)