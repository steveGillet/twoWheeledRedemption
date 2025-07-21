import os
import sys
import time
import smbus
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick

address =0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

print("Calibrating Gyro....")
imu.caliberateGyro()
print("Calibration Complete.")

sensorfusion = madgwick.Madgwick(0.5)

currTime = time.time()
printCount = 0
try:
    while True:
        imu.readSensor()
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime

        sensorfusion.updateRollPitchYaw(
            imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2],
            imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],
            imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt
        )

        if printCount == 2:
            print("Roll: {0:.2f} Pitch: {1:.2f} Yaw: {2:.2f}".format(
                sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw
            ))
            printCount = 0
        printCount += 1

        time.sleep(0.01)

except KeyboardInterrupt:
    print("you are terminated")
    sys.exit()