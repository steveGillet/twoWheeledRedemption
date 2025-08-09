import os
import sys
import time
import smbus
import numpy as np
from imusensor.MPU9250 import MPU9250
from imusensor.filters import madgwick
from scipy.io import loadmat
from control import ss, c2d
from gpiozero import Motor, PWMOutputDevice, DigitalInputDevice

rightMotor = Motor(13, 6)
leftMotor = Motor(19, 26)
rightMotorpwm = PWMOutputDevice(20)
leftMotorpwm = PWMOutputDevice(21)

class Encoder:
    def __init__(self, pinNumberA, pinNumberB):
        self.counter = 0
        self.encoderInputA = DigitalInputDevice(pinNumberA)
        self.encoderInputA.when_activated = self.cycleCount
        self.encoderInputB = DigitalInputDevice(pinNumberB)
        self.direction = None

    def cycleCount(self):
        self.counter += 1
        if (self.encoderInputB.value == 0):
            self.direction = "backward"
        else:
            self.direction = "forward"
        if (self.counter >= 1120):
            print("One cycle completed")
            self.counter = 0
            print("I am moving ", self.direction)

rightEncoder = Encoder(14, 15)

# IMU setup
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

print("Calibrating Gyro....")
imu.caliberateGyro()  # Run once; comment out after if stable
# Optional: Calibrate accel/mag if needed
# imu.caliberateAccelerometer()
# imu.caliberateMag()
# imu.saveCalibDataToFile('/home/ubuntu/calib.json')  # Save for future loads
# imu.loadCalibDataFromFile('/home/ubuntu/calib.json')
print("Calibration Complete.")

sensorfusion = madgwick.Madgwick(0.5)  # Beta=0.5; adjust if needed for smoothness vs. accuracy

# Controller setup
data = loadmat('controller.mat')
K_A = np.squeeze(data['K_A'])
K_B = np.squeeze(data['K_B'])
K_C = np.squeeze(data['K_C'])
K_D = np.squeeze(data['K_D'])
K = ss(K_A, K_B, K_C, K_D)

Ts = 0.01  # Sampling time (s)
Kd = c2d(K, Ts, method='tustin')
Ad, Bd, Cd, Dd = Kd.A, Kd.B, Kd.C, Kd.D
n_states = Ad.shape[0]
x_k = np.zeros((n_states, 1))  # Initial controller state

# Offsets (calibrate these!)
pitch_offset = 4.0  # Degrees; adjust based on upright reading after inversion
yaw_initial = 0.0   # Set in loop for relative yaw

# Reference (setpoint)
r = np.array([[0.0], [0.0]])  # theta=0 (upright), psi=0 (desired yaw)

# Motor control function (integrated and adapted from your code, using separate PWM)
def apply_control(u):
    # u is (2,1) array: u[0] for common mode (forward/tilt control), u[1] for diff mode (yaw control)
    # Scale u to motor speeds: Assume u in [-max_u, max_u] maps to [-1,1] speed
    print('raw u', u)
    max_u = 0.1  # Tune this based on your controller output range (e.g., from sims)
    u_scaled = u / max_u
    print('scaled u', u_scaled)
    left_speed = np.clip(u_scaled[0] + u_scaled[1], -1.0, 1.0)  # Common + diff
    right_speed = np.clip(u_scaled[0] - u_scaled[1], -1.0, 1.0)  # Common - diff (adjust sign if yaw direction wrong)

    # Set directions and PWM values, COMMENT THIS OUT FOR TEST
    if left_speed > 0:
        leftMotor.forward()
        leftMotorpwm.value = left_speed
    elif left_speed < 0:
        leftMotor.backward()
        leftMotorpwm.value = -left_speed  # abs for PWM
    else:
        leftMotor.stop()
        leftMotorpwm.value = 0.0

    if right_speed > 0:
        rightMotor.forward()
        rightMotorpwm.value = right_speed
    elif right_speed < 0:
        rightMotor.backward()
        rightMotorpwm.value = -right_speed  # abs for PWM
    else:
        rightMotor.stop()
        rightMotorpwm.value = 0.0
    
    # Optional: Print for debug
    print(f"Control: left_speed={left_speed:.2f}, right_speed={right_speed:.2f}")

# Main control loop
currTime = time.time()
first_run = True
try:
    while True:
        start_time = time.time()

        # Read IMU
        imu.readSensor()
        newTime = time.time()
        dt = newTime - currTime
        currTime = newTime

        # Handle inverted mounting: Invert z-axis for NED alignment
        imu.AccelVals[2] = -imu.AccelVals[2]
        imu.GyroVals[2] = -imu.GyroVals[2]
        imu.MagVals[2] = -imu.MagVals[2]

        # Update fusion (run multiple times for stability if dt is small)
        for _ in range(5):  # Adjust (e.g., 10) for smoother output
            sensorfusion.updateRollPitchYaw(
                imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2],
                imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2],
                imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt / 5
            )

        # Get angles in degrees, apply corrections
        pitch = sensorfusion.pitch - pitch_offset  # Theta (tilt)
        yaw = sensorfusion.yaw
        if first_run:
            yaw_initial = yaw  # Set initial yaw as 0 reference
            first_run = False
        yaw_relative = yaw - yaw_initial  # Psi (relative yaw)
        print(f"Pitch: {pitch:.2f}, Yaw: {yaw_relative:.2f}")

        # Convert to radians for controller
        theta_rad = np.deg2rad(pitch)
        psi_rad = np.deg2rad(yaw_relative)

        # Sensor output y (shape (2,1))
        y = np.array([[theta_rad], [psi_rad]])

        # Control computation
        e = r - y  # Error
        u = Cd @ x_k + Dd @ e
        x_k = Ad @ x_k + Bd @ e

        # Apply to hardware
        apply_control(u)

        # Maintain Ts
        elapsed = time.time() - start_time
        if elapsed < Ts:
            time.sleep(Ts - elapsed)

except KeyboardInterrupt:
    print("Terminated")
    # Stop motors on exit
    rightMotor.stop()
    leftMotor.stop()
    rightMotorpwm.value = 0.0
    leftMotorpwm.value = 0.0
    sys.exit()