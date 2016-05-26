from __future__ import division
import sys
import time
import serial
import math
import cv2
import numpy as np
import random


################################################################################
# Hardware
################################################################################

cap = cv2.VideoCapture(0)
ser = serial.Serial(port = '/dev/ttyACM0', baudrate = 921600, timeout = None)


################################################################################
# Constants
################################################################################

above_board_tilt = 0.73
rod_down_tilt = 0.85
raise_fish_tilt = 0.55
shake_low_tilt = 0.9
shake_high_tilt = 0.7

################################################################################
# Globals
################################################################################

x_last = 0
pan_last = 0
tilt_last = raise_fish_tilt
robot_data = {'ack': False, 'hlim': False, 'llim': False, 'start': False, 'caught': False, 'pushing': False}


################################################################################
# Functions
################################################################################

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def command(x, dx, pan, dpan, tilt, dtilt, home=False):
    global x_last, pan_last, tilt_last, robot_data
    x_last = x
    pan_last = pan
    tilt_last = tilt

    x = clamp(int(x*65535), 0, 65534);
    xh = x >> 8 & 0xff
    xl = x & 0xff

    dx = clamp(int(dx*255), 0, 254)

    pan = clamp(int(pan*32767), -32767, 32767);
    panh = pan >> 8 & 0xff
    panl = pan & 0xff
    if panh < 0:
        panh = 255 - panh

    tilt = clamp(int(tilt*32767), -32767, 32767);
    tilth = tilt >> 8 & 0xff
    tiltl = tilt & 0xff
    if tilth < 0:
        tilth = 255 - tilth

    dpan = clamp(int(dpan/10.0*127), -127, 127)
    if dpan < 0:
        dpan = 255 + dpan
    dtilt = clamp(int(dtilt/10.0*127), -127, 127)
    if dtilt < 0:
        dtilt = 255 + dtilt

    flags = 0x00
    if home:
        flags = flags | 0x01

    checksum = (xh + xl + dx + panh + panl + tilth + tiltl + dtilt + dpan + flags) & 0x7f

    ser.write(bytearray([0xff, 0xff, dx, xh, xl, tilth, tiltl, panh, panl, dtilt, dpan, flags, checksum]))
    if ser.in_waiting:
        return_code = ord(ser.read(1))
        robot_data['ack'] = bool(return_code & 0x01)
        robot_data['hlim'] = bool(return_code & 0x02)
        robot_data['llim'] = bool(return_code & 0x04)
        robot_data['start'] = bool(return_code & 0x08)
        robot_data['caught'] = bool(return_code & 0x10)
        robot_data['pushing'] = bool(return_code & 0x20)

    ser.reset_input_buffer()

    return robot_data


def move_with_speed(x, x_speed, pan, pan_speed, tilt, tilt_speed):
    start_time = time.time()
    time_last = time.time()
    dt = 1/100

    x_time = math.fabs((x - x_last) / x_speed)
    pan_dir = math.copysign(1, pan - pan_last)
    tilt_dir = math.copysign(1, tilt - tilt_last)

    pan_speed = math.copysign(pan_speed, pan_dir)
    tilt_speed = math.copysign(tilt_speed, tilt_dir)

    # Send a command update once every dt periof
    while (pan - pan_last) * pan_dir > 0 or (tilt - tilt_last) * tilt_dir > 0:
        if (pan - pan_last) * pan_dir > 0:
            pan_new = pan_last + pan_speed * dt
        else:
            pan_new = pan

        if (tilt - tilt_last) * tilt_dir > 0:
            tilt_new = tilt_last + tilt_speed * dt
        else:
            tilt_new = tilt;

        while time.time() - time_last < dt:
            pass
        time_last = time.time()
        command(x, x_speed, pan_new, pan_speed, tilt_new, tilt_speed)

    # Wait until the x axis is done
    while time.time() - start_time < x_time:
        command(x, x_speed, pan, 0, tilt, 0)

def poll_robot_data():
    return command(x_last, 0, pan_last, 0, tilt_last, 0)


def pixel_to_x_pan(pixel):
    xp, yp = pixel
    sqr_val = 1.0 - 0.000025*((0.33043151*yp - 0.0020920432*xp + 6.6824307)/(0.00001608983*xp + 0.00043866784*yp - 1.0) + 2.0)**2
    if sqr_val >= 0:
        x = 1.3417323*math.sqrt(sqr_val) + (0.0067086614*(0.35185384*xp + 0.017212861*yp - 135.47421))/(0.00001608983*xp + 0.00043866784*yp - 1.0) - 0.77133228
    else:
        x = float('nan')

    sin_val = (0.005*(0.33043151*yp - 0.0020920432*xp + 6.6824307))/(0.00001608983*xp + 0.00043866784*yp - 1.0) + 0.01
    if sin_val <= 1 and sin_val >= -1:
        pan = 0.53475936*math.asin(sin_val) - 0.015
    else:
        pan = float('nan')
    return (x, pan)


def valid_x_pan(x, pan):
    if math.isnan(x) or x < 0 or x > 1:
        return False
    if math.isnan(pan) or pan < -1 or pan > 1:
        return False
    return True


################################################################################
# Start main program
################################################################################

# Home the x axis
print 'Homing x axis...'
ret = command(0, 1, -0.6, 0, raise_fish_tilt, 0, True)
while not ret['llim']:
    ret = command(0, 1, -0.6, 0, raise_fish_tilt, 0, True)
print 'Homing complete'

# Move out of the way of the camera
move_with_speed(1, 0.3, -0.6, 1, above_board_tilt, 0.1)

# Take a picture of the board
print 'Imaging board...'
_, frame = cap.read()

# Find the fishing spots marked in red
print 'Extracting fishing spots...'

# Get red pixels
hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
lower_magenta = np.array([170, 150, 80], dtype = 'uint8')
upper_magenta = np.array([179, 255, 255], dtype = 'uint8')
lower_orange = np.array([0, 150, 80], dtype = 'uint8')
upper_orange = np.array([10, 255, 255], dtype = 'uint8')
mask_magenta = cv2.inRange(hsv_frame, lower_magenta, upper_magenta)
mask_orange = cv2.inRange(hsv_frame, lower_orange, upper_orange)
mask_red = cv2.add(mask_magenta, mask_orange)
mask_red_blur = cv2.blur(mask_red, (5, 5))

# Find blobs
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = True
params.blobColor = 255
params.filterByArea = True
params.minArea = 20
params.maxArea = 150
params.filterByConvexity = True
params.minConvexity = 0.8
params.filterByCircularity = False
detector = cv2.SimpleBlobDetector(params)
keypoints = detector.detect(mask_red)

frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# cv2.imshow('Fishing spots', frame_with_keypoints)
# cv2.waitKey(100)

fishing_spots = [];
for keypoint in keypoints:
    # Test whether point is within workspace
    (x, pan) = pixel_to_x_pan(keypoint.pt)
    if valid_x_pan(x, pan):
        fishing_spots.append(keypoint.pt)
print 'Found {} fishing spots'.format(len(fishing_spots))

# Wait for game to start
print 'Waiting for start button...'
while not robot_data['start']:
    command(x_last, 0, pan_last, 0, tilt_last, 0)
    time.sleep(0.1)
print 'Starting!'

# Main fishing loop
while robot_data['start']:
    # Pick a random fishing spot and go there
    print 'Choosing a new spot...'
    spot = random.choice(fishing_spots)
    (x, pan) = pixel_to_x_pan(spot)
    move_with_speed(x, 0.1, pan, 0.1, above_board_tilt, 0.1)

    # Try to catch fish here up to some number of times
    for _ in xrange(5):
        # Break if the start button was turned off
        if not robot_data['start']:
            break

        # Dip rod
        move_with_speed(x, 0.1, pan, 0.1, rod_down_tilt, 0.3)

        # If rod is pushing against something, it didn't go into a fish
        if robot_data['pushing']:
            move_with_speed(x, 0.1, pan, 0.1, above_board_tilt, 0.1)
            time.sleep(1)
        else:
            # Rod is in a fish, so wait a bit and pick it up
            time.sleep(1)
            move_with_speed(x, 0.1, pan, 0.1, raise_fish_tilt, 0.3)
            time.sleep(0.3)

            # Check whether the fish was caught
            poll_robot_data()
            if robot_data['caught']:
                print 'Caught a fish!'

                # Bring the fish to the goal area
                move_with_speed(0, 0.2, pan, 0.3, raise_fish_tilt, 0.3)
                move_with_speed(0, 0.2, 0.3, 0.3, raise_fish_tilt, 0.3)

                # Shake some number of times or until free
                for _ in xrange(10):
                    print 'Released fish!'
                    move_with_speed(0, 0.2, 0.3, 0.3, shake_low_tilt, 1)
                    move_with_speed(0, 0.2, 0.3, 0.3, shake_high_tilt, 1)
                    move_with_speed(0, 0.2, 0.3, 0.3, shake_low_tilt, 1)
                    move_with_speed(0, 0.2, 0.3, 0.3, shake_high_tilt, 1)
                    move_with_speed(0, 0.2, 0.3, 0.3, raise_fish_tilt, 1)
                    time.sleep(0.3)
                    poll_robot_data()
                    if not robot_data['caught']:
                        break

                # Return to main workspace
                move_with_speed(0, 0.2, -0.3, 0.1, raise_fish_tilt, 0.1)
                break

# Get out of the way when finished
print 'Stopping...'
move_with_speed(1, 0.3, -0.6, 1, above_board_tilt, 0.1)

# OpenCV cleanup
cap.release()
cv2.destroyAllWindows()
