import pygame
import sys
import time
import serial
import math

# configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=921600,
    timeout=None
)
pygame.init()

pygame.display.set_mode((100, 100))

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def command(x, dx, pan, dpan, tilt, dtilt, home):
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
    else:
        return_code = 0
    ser.reset_input_buffer()

    return {'hlim': bool(return_code & 0x02), 'llim': bool(return_code & 0x04), 'start': bool(return_code & 0x08), 'catch': bool(return_code & 0x01)}


x = 0;
pan = -0.3
tilt = 0.8
tiltn = 0.7
tiltl = 0.6
tilth = 0.8

dt = 1/60.0
panlast = 0
tiltlast = 0
xlast = 0
dx = 0

command(0, 0, pan, 0, tilt, 0, True)

while True:
    keyState = pygame.key.get_pressed()
    panlast = pan
    tiltlast = tilt
    xlast = x

    factor = 1
    if keyState[pygame.K_LSHIFT]:
        factor = 0.1

    if keyState[pygame.K_LEFT] and x < 1:
        x += 0.01*factor
    if keyState[pygame.K_RIGHT] and x > 0:
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

    xdiff = abs(x - xlast)
    if xdiff > 0:
        dx = xdiff/dt * 0.45

    ret = command(x, dx, pan, dpan, tilt, dtilt, False)
    # print 'x={}, pan={}'.format(x, pan)
    # print ret


    time.sleep(dt)

    pygame.event.pump()

