from gpiozero import Motor, PWMOutputDevice, DigitalInputDevice
import time

rightMotor = Motor(13,6)
leftMotor = Motor(19,26)
rightMotorpwm = PWMOutputDevice(20)
leftMotorpwm = PWMOutputDevice(21)


def moveForward():
    rightMotor.forward()
    leftMotor.forward()
    rightMotorpwm.on()
    leftMotorpwm.on()

def stop():
    rightMotor.stop()
    leftMotor.stop()
    rightMotorpwm.off()
    leftMotorpwm.off()

def arcRight():
    rightMotor.forward()
    leftMotor.forward()
    rightMotorpwm.value = 0.1
    leftMotorpwm.on()

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

print("I'm moving forward!")
moveForward()
time.sleep(5)
print("I'm stopping!")
stop()