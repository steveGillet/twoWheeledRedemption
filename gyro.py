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
print("Calibrating Magnetometer (move in figure-8 for 30-60s)....")
imu.caliberateMagApprox()   # Run this; move sensor in figure-8
imu.caliberateAccelerometer()  # Also calib accel for completeness
print("Calibration Complete.")
imu.saveCalibDataToFile('/home/steve/Desktop/twoWheeledRedemption/calib.json')  # Save for future runs
# imu.loadCalibDataFromFile('/home/steve/Desktop/twoWheeledRedemption/calib.json')

sensorfusion = madgwick.Madgwick(0.5)

currTime = time.time()
printCount = 0
try:
    while True:
        imu.readSensor()

        imuAccelX = imu.AccelVals[1]
        imuAccelY = imu.AccelVals[0]
        imuAccelZ = -imu.AccelVals[2]

        imuGyroX = imu.GyroVals[1]
        imuGyroY = imu.GyroVals[0]
        imuGyroZ = -imu.GyroVals[2]
        
        imuMagX = imu.MagVals[1]
        imuMagY = imu.MagVals[0]
        imuMagZ = -imu.MagVals[2]

        for i in range(10):
            newTime = time.time()
            dt = newTime - currTime
            currTime = newTime
            sensorfusion.updateRollPitchYaw(
                imuAccelX, imuAccelY, imuAccelZ,
                imuGyroX, imuGyroY, imuGyroZ,
                imuMagX, imuMagY, imuMagZ, dt
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