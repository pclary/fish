import pygame
import sys
import time
import serial
import math
import cv2

camera = cv2.VideoCapture(0)
camera.set(cv2.cv.CV_CAP_PROP_FPS, 60)

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=921600
)
pygame.init()

pygame.display.set_mode((100, 100))

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def command(x, dx, pan, dpan, tilt, dtilt, home):
    x = clamp(int(pan*65535), 0, 65534);
    xh = x >> 8 & 0xff
    xl = x & 0xff

    dx = clamp(int(x*255), 0, 254)

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
        flags = flags & 0x01

    checksum = (xh + xl + dx + panh + panl + tilth + tiltl + dtilt + dpan + flags) & 0x7f
    ser.write(bytearray([0xff, 0xff, dx, xh, xl, tilth, tiltl, panh, panl, dtilt, dpan, flags, checksum]))


x = 0;
pan = -0.3
tilt = 0.8
tiltn = 0.8
tiltl = 0.7
tilth = 0.9

dt = 1/60.0
panlast = 0
tiltlast = 0
xlast = 0

command(0, 0, 0, 0, 0, 0, True)

while True:
    keyState = pygame.key.get_pressed()
    panlast = pan
    tiltlast = tilt
    xlast = x

    factor = 1
    if keyState[pygame.K_LSHIFT]:
        factor = 0.03

    if keyState[pygame.K_LEFT]:
        x += 0.01*factor
    if keyState[pygame.K_RIGHT]:
        x -= 0.01*factor
    if keyState[pygame.K_UP] and pan < 1:
        pan += 0.005*factor
    if keyState[pygame.K_DOWN] and pan > -1:
        pan -= 0.005*factor
    if keyState[pygame.K_z] and tilt < tilth:
        tilt += 0.01*factor
    if keyState[pygame.K_x] and tilt > tiltl:
        tilt -= 0.01*factor

    if not keyState[pygame.K_z] and not keyState[pygame.K_x]:
        if tilt > tiltn:
            tilt -= 0.01
        if tilt < tiltn:
            tilt += 0.01

    if keyState[pygame.K_ESCAPE]:
        break

    dtilt = (tilt - tiltlast)/dt
    dpan = (pan - panlast)/dt
    dx = 0.2

    command(x, dx, pan, dpan, tilt, dtilt, False)

    ret, frame = camera.read()
    cv2.imshow('fish', frame)
    cv2.waitKey(1)
    # time.sleep(dt)

    pygame.event.pump()

camera.release()
cv2.destroyAllWindows()
