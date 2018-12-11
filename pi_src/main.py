
# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
from ultrasonic_distance import UltraSonicSensor
import numpy as np
import imutils
import time
import cv2
from buckler_telnet import BucklerTelnet


# TODO: Change to True in production
use_buckler_rtt = True

# Setup PiCamera
print("[INFO] Setting up Camera...")
#camera = cv2.VideoCapture(0)
camera = PiCamera()
rawCapture = PiRGBArray(camera)
time.sleep(1.0)
fps = FPS().start()
print("[INFO] Camera Setup complete.")

# Setup Ultrasonic Sensor
ultraSonicSensor = UltraSonicSensor()
ultraSonicSensor.setup(7, 11)

# PS: (0-360, 0-100, 0-100) -> CV2: (0-180, 0-255, 0-255)
RED_LOWER = (353 // 2, 68 * 255 // 100, 35 * 255 // 100)
RED_UPPER = (360 // 2, 85 * 255 // 100, 75 * 255 // 100)

# Set image sampling frequency
time_per_frame = 1 / 20
last_update = time.time()
last_cup_time = 0

# Set frame width and height
IMG_WIDTH = 600
IMG_HEIGHT = 500

if use_buckler_rtt:
    # Set up RTT Buckler Comm
    bucklerRTT = BucklerTelnet()
    time.sleep(1.0)
    # Reset grabber and lift actuators
    bucklerRTT.liftCup()
    bucklerRTT.resetGrabber()
    bucklerRTT.setSpeed("50")

# Set up PiCam Constants
PIXEL_WIDTH = 0.000112  # in cm
# FOCAL_LENGTH = 0.304  # in cm
FOCAL_LENGTH = 2533  # could also be 3.04 / 0.0012 = 2533 or 4.01
# Set up cup distance constants
CUP_WIDTH = 8.128  # in cm

def distance_to_camera(knownWidth, focalLength, radius):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / radius

def detect_cup(frame, min_cup_radius=10):
    """
    Finds and returns the largest red cup's center pixel coordinates and pixel radius.
    Args:
        - frame (bgr picam image)
    Returns:
        - cup_found (boolean) : True if cup_found, else False
        - cup_center (tuple(int, int)) : pixel coordinates (u, v) of cup's center using blob.
        - cup_radius (int) : radius of the cup blob in pixels.
    """
    # Convert bgr image frame to hsv color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Apply Red color mask
    mask = cv2.inRange(hsv, RED_LOWER, RED_UPPER)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    radius = None
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((u, v), radius) = cv2.minEnclosingCircle(c)

    if len(cnts) > 0 and radius >= min_cup_radius:
        last_ball_time = time.time()
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(frame, (int(u), int(v)), int(radius), (0, 255, 255), 2)
    return center is not None, center, radius


def center_cup(cup_center, cup_width, center_threshold=10):
    """
    Obtains the deviation of the cup's pixel coordinates (u, v) from the horizontal image center.
    If deviation above some threshold, sends RTT command over telnet_lib.
    Args:
        - cup_center (tuple(int, int)) : cup center in pixel coordinates
        - cup_radius (int) : cup_radius in integer number of pixels
        - center_threshold (int) : max number of pixels from the center the cup can deviate.
    Returns:
        - centered (boolean) : returns True if the cup has been centered.
    """
    # Get deviation from the image center width.
    u, v = cup_center
    # right_deviation = (u - cup_width) - (IMG_WIDTH / 2)
    # left_deviation = (IMG_WIDTH / 2) - (u + cup_width)
    #if (u - cup_width) > (IMG_WIDTH // 2):  # on the right side of the center
    #    deviation = u - cup_width - (IMG_WIDTH // 2)
    #elif (u + cup_width) < (IMG_WIDTH // 2):  # on the left side of the center
    #    deviation = (IMG_WIDTH // 2) - (u + cup_width)
    #else:   # cup_width straddles the center
    #    deviation = 0
    deviation = u - (IMG_WIDTH // 2)
    target_angle = 4
    if cup_width >= 25 and abs(deviation) <= cup_width + center_threshold:
        # Assume cup is centered
        deviation = center_threshold
        target_angle = 0

    cup_dist = ultraSonicSensor.get_distance()
    if cup_dist > 60 and cup_dist < 2000:
        print("Correcting distance before centering")
        correct_distance(cup_dist, 15, 45)
    print("Deviation: ", deviation)
    print("Target Angle: ", target_angle)
    # Calculate fuzzy target angle
    # Z = distance_to_camera(CUP_WIDTH, FOCAL_LENGTH, cup_width)
    # print("Camera distance: ", Z)
    # target_angle = max(np.rad2deg(np.arctan(abs(deviation) * PIXEL_WIDTH / Z)), target_angle)
    print("Target angle: ", target_angle)
    if abs(deviation) <= center_threshold:
        # cup is centered send no command.
        return True
    elif deviation > 0:  # Cup is too far to the right of the camera/robot system
        print("Sending RTT Right")
        if use_buckler_rtt:
            bucklerRTT.turnRightAngle(target_angle)
        return False
    else:  # Cup is too far to the left of the camera/robot system
        print("Sending RTT Left")
        if use_buckler_rtt:
            bucklerRTT.turnLeftAngle(target_angle + 4)
        return False

def correct_distance(cup_distance, min_cup_dist=10, max_cup_dist=20):
    """
    Runs RTT command forward or backward if the cup is not in pickup distance range.
    Returns:
        - in_pickup_range (boolean) : True if cup within pickup range else False
    """
    if min_cup_dist <= cup_distance and cup_distance <= max_cup_dist:
        # TODO: We are within the pick up range
        # Run pickup sequence
        return True
    elif cup_distance < min_cup_dist:
        # Move backward
        # Send RTT
        print("Sending reverse command", (min_cup_dist + max_cup_dist) / 2 - cup_distance)
        if use_buckler_rtt:
            bucklerRTT.reverseDist(max(min(((min_cup_dist + max_cup_dist) / 2 - cup_distance) / 100.0, 0.25), 0.12))
        return False
    elif cup_distance > max_cup_dist:
        # Move forward
        # Send RTT
        print("Sending forward command", min(cup_distance - (min_cup_dist + max_cup_dist) / 2 / 100.0, 0.1))
        if use_buckler_rtt:
            bucklerRTT.driveDist(max(min((cup_distance - (min_cup_dist + max_cup_dist) / 2) / 100.0, 0.25), 0.08))
        return False

def pickup_cup():
    """
    Assumes the cup is within pickup range.
    Sends the pick sequence command over Buckler RTT.
    """
    if use_buckler_rtt:
        print("Sending Pickup Sequence")
        bucklerRTT.resetLift()
        bucklerRTT.rotateGrabber()
        bucklerRTT.liftCup()
        bucklerRTT.resetGrabber()

def avoid_obstacle():
    """
    Avoid Obstacle movement sequence
    """
    print("Avoiding obstacle")
    if use_buckler_rtt:
        bucklerRTT.reverseDist(0.15)
    print("Avoid turn")
    if use_buckler_rtt:
        bucklerRTT.turnLeftAngle(15)

def scan(counter):
    print("Attempting scan count: ", counter)
    if counter < 72:
        print("Scanning around")
        if use_buckler_rtt:
            bucklerRTT.turnRightAngle(8)
    counter += 1
    return counter

def main():
    """
    Continuous Raspberry Pi Main Detection Loop.
    Algorithm:
    - While True:
        - Try detecting a Red Cup in the current camera scene.

        - If blob is not None or blob is large enough:

            - Try to center the blob center in the scene (using image width and height) 
                - send Left/Right commands to buckler over RTT (telnet_lib.py) 
            - If centered:
                - Use the ultrasonic sensor to get a high resolution distance.
                OR
                - Get the distance away from the cup using a pinhole camera model with the width of the cup in pixels, focal length,
            and expected cup width.

                - If distance within calibrated pickup range:
                    - send pickup command sequence over telnet_lib.

        - Else:
            - Continue Spiraling outward from current position
                - Send Telnet_lib command
    """

    i = 0  # Delete in production

    scan_counter = 0  # Tracks number of times we have turned right in a scan, resets when cup is found.
    frame_since_found_cup = 1000

    # Continuously stream camera frames.
    #while True:
    for frame in camera.capture_continuous(rawCapture, format="bgr",  use_video_port=True):
        # Skip frame if too fast.
        if time.time() - last_update < time_per_frame:
            time.sleep(time_per_frame / 5)
            continue

        # grab the frame from the stream and resize it to have a maximum width of 400 pixels
        lastUpdate = time.time()
        # Flip the image vertically (if PiCam is upside down)
        frame = cv2.flip(frame.array, 1)
        #grabbed, frame = camera.read()
        #cv2.imshow('image', np.array(frame, dtype=np.uint8))
        frame = imutils.resize(frame, width=IMG_WIDTH, height=IMG_HEIGHT)

        # Detect cup from frame
        min_cup_width = 3  # defines a cup blob width in pixels
        found_cup, cup_center, cup_width = detect_cup(frame, min_cup_width)
        print("[INFO] found cup status: ", found_cup, cup_center, cup_width)
        if found_cup:
            frame_since_found_cup = 0
            scan_counter = 0
            # Visualize the cup frame
            # cv2.imshow("Cup", np.array(frame, dtype=np.uint8))
            # cv2.waitKey(1) & 0xFF
            # Try to center the cup
            max_cup_center_deviation = 30  # in pixels
            cup_centering_complete = center_cup(cup_center, cup_width, max_cup_center_deviation)
            if cup_centering_complete:

                # Try to update the robot's distance from the cup.
                cup_distance = ultraSonicSensor.get_distance()
                min_cup_dist = 20  # in cm
                max_cup_dist = 31  # in cm
                distance_corrected = correct_distance(cup_distance, min_cup_dist, max_cup_dist)
                if distance_corrected:
                    pickup_cup()
                    i = 0
                else:
                    # Robot is currently correcting its distance from cup over RTT
                    pass

            else:
                # Cup is currently being centered over RTT.
                pass

        else:
            frame_since_found_cup += 1
            # Try Obstacle detection sequence using ultrasonic sensor distance
            obstacle_distance = ultraSonicSensor.get_distance()
            obstacle_threshold = 50  # in cm
            if obstacle_distance < obstacle_threshold and frame_since_found_cup > 3:
                # Run obstacle detection sequence
                avoid_obstacle()
                i = 0
            elif scan_counter < 72:
                # Run full rotation scan.
                scan_counter = scan(scan_counter)
            else:
                # TODO: Continue spiral movement path
                print("Sending spiral command", i + 1)
                for x in range(i + 1):
                    print("Sending spiral command")
                    bucklerRTT.driveDist(0.1)
                bucklerRTT.turnRightAngle(10)
                i += 1

        rawCapture.truncate(0)


# If there is an error in the script, cleanup objects.
try:
    main()
except KeyboardInterrupt:
    print("Cleaning up.")
    camera.release()
    cv2.destroyAllWindows()
    ultraSonicSensor.destroy()
