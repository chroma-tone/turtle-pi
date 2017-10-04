import RPi.GPIO as GPIO
from time import sleep

class ShiftRegisterMotorControl:
    # Static state of latch across all ShiftRegisterMotorControl instances
    latchState = 0
    gpiosInitialized = False

    # Bit positions in the 74HCT595 shift register output
    MOTOR1_A = 2
    MOTOR1_B = 3
    MOTOR2_A = 1
    MOTOR2_B = 4
    MOTOR3_A = 5
    MOTOR3_B = 7
    MOTOR4_A = 0
    MOTOR4_B = 6

    # Constants that the user passes in to the motor calls
    FORWARD = 1
    BACKWARD = 2
    BRAKE = 3
    RELEASE = 4

    # RPI pins: Ref https://www.element14.com/community/servlet/JiveServlet/previewBody/73950-102-11-339300/pi3_gpio.png
    # Interface to 74HCT595 latch

# Purpose  74HC595  Arduino  Rpi     Color
# !Latch   12       D12      GPIO19  White
# !CLK     11       D4       GPIO13  Grey
# !EN      13       D7       GPIO6   Purple
# DATA     14       D8       GPIO5   Blue

    MOTORLATCH = 19 
    MOTORCLK = 13 
    MOTORENABLE = 6 
    MOTORDATA = 5 

    # PWM motor speed control
    MOTOR1_EN = 12 # Arduino D11
    MOTOR2_EN = 16 # Arduino D3
    MOTOR3_EN = 20 # Arduino D5
    MOTOR4_EN = 21 # Arduino D6

    # PWM control frequency (Hz)
    PWM_FREQ = 100

    @staticmethod
    def initializeGpios():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ShiftRegisterMotorControl.MOTORLATCH, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTORCLK, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTORENABLE, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTORDATA, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR1_EN, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR2_EN, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR3_EN, GPIO.OUT)
        GPIO.setup(ShiftRegisterMotorControl.MOTOR4_EN, GPIO.OUT)

        GPIO.output(ShiftRegisterMotorControl.MOTORENABLE, GPIO.LOW)
        ShiftRegisterMotorControl.gpiosInitialized = True

    def __init__(this, motorNumber):
        if motorNumber > 4 or motorNumber < 1:
            raise ValueError, "motorNumber must be between 1 and 4 inclusive"

        if ~ShiftRegisterMotorControl.gpiosInitialized:
            ShiftRegisterMotorControl.initializeGpios()

        if motorNumber == 1:
            this.motorA = ShiftRegisterMotorControl.MOTOR1_A
            this.motorB = ShiftRegisterMotorControl.MOTOR1_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR1_EN, ShiftRegisterMotorControl.PWM_FREQ)

        elif motorNumber == 2:
            this.motorA = ShiftRegisterMotorControl.MOTOR2_A
            this.motorB = ShiftRegisterMotorControl.MOTOR2_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR2_EN, ShiftRegisterMotorControl.PWM_FREQ)

        elif motorNumber == 3:
            this.motorA = ShiftRegisterMotorControl.MOTOR3_A
            this.motorB = ShiftRegisterMotorControl.MOTOR3_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR3_EN, ShiftRegisterMotorControl.PWM_FREQ)

        elif motorNumber == 4:
            this.motorA = ShiftRegisterMotorControl.MOTOR4_A
            this.motorB = ShiftRegisterMotorControl.MOTOR4_B
            this.motorPwm = GPIO.PWM(ShiftRegisterMotorControl.MOTOR4_EN, ShiftRegisterMotorControl.PWM_FREQ)

        # Initialize motor driver to idle state
        this.motorPwm.start(0)
        this.setDirection(ShiftRegisterMotorControl.RELEASE)

    @staticmethod
    def latch_tx():
        GPIO.output(ShiftRegisterMotorControl.MOTORLATCH, GPIO.HIGH)

        print "ShiftRegisterMotorControl.latchState = " + str(ShiftRegisterMotorControl.latchState)
        for i in range(0,8):
            
            # Set up data pin with next bit
            if (ShiftRegisterMotorControl.latchState << i) & 0x80 == 0x80:
                print "Latching 1"
                GPIO.output(ShiftRegisterMotorControl.MOTORDATA, GPIO.HIGH)
            else:
                print "Latching 0"
                GPIO.output(ShiftRegisterMotorControl.MOTORDATA, GPIO.LOW)

            # And pulse clock to serialize through
            GPIO.output(ShiftRegisterMotorControl.MOTORCLK, GPIO.LOW)
            sleep(0.001)
            GPIO.output(ShiftRegisterMotorControl.MOTORCLK, GPIO.HIGH)
            sleep(0.001)

        GPIO.output(ShiftRegisterMotorControl.MOTORLATCH, GPIO.LOW)

    def setDirection(this, direction):
        if direction == ShiftRegisterMotorControl.FORWARD:
            ShiftRegisterMotorControl.latchState |= 1 << this.motorA
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorB)

        if direction == ShiftRegisterMotorControl.BACKWARD:
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorA)
            ShiftRegisterMotorControl.latchState |= 1 << this.motorB

        if direction == ShiftRegisterMotorControl.RELEASE:
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorA)
            ShiftRegisterMotorControl.latchState &= ~(1 << this.motorB)

        ShiftRegisterMotorControl.latch_tx()

    def setSpeed(this, percentSpeed):
        print "Changing duty cycle to " + str(percentSpeed)
        this.motorPwm.ChangeDutyCycle(percentSpeed)

def test1():
    motor1 = ShiftRegisterMotorControl(1)
    motor1.setSpeed(100)

    for i in range(1,10):
        print "Forward"
        motor1.setDirection(ShiftRegisterMotorControl.FORWARD)
        sleep(10)

        print "Backwards"
        motor1.setDirection(ShiftRegisterMotorControl.BACKWARD)
        sleep(10)

        print "Release"
        motor1.setDirection(ShiftRegisterMotorControl.RELEASE)
        sleep(10)

    motor1.setSpeed(0)

try:
    ShiftRegisterMotorControl.initializeGpios()
    while True:
        for i in range(0,8):
            ShiftRegisterMotorControl.latchState = 1 << i
            ShiftRegisterMotorControl.latch_tx()
            sleep(1)
except:
    pass

# test1()
GPIO.cleanup()
