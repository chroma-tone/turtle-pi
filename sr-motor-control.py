import Rpi.GPIO as GPIO
import Rpi.GPIO.LOW as LOW
import Rpi.GPIO.HIGH as HIGH

class ShiftRegisterMotorControl:
    # Static state of latch across all ShiftRegisterMotorControl instances
    latchState = 0

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
    MOTORLATCH = 5 # Arduino D12
    MOTORCLK = 6 # Arduino D4
    MOTORENABLE = 13 # Arduino D7
    MOTORDATA = 19 # Arduino D8

    # PWM motor speed control
    MOTOR1_EN = 12 # Arduino D11
    MOTOR2_EN = 16 # Arduino D3
    MOTOR3_EN = 20 # Arduino D5
    MOTOR4_EN = 21 # Arduino D6

    # PWM control frequency (Hz)
    PWM_FREQ = 100

    def __init__(this, motorNumber):
        if motorNumber > 4 || motorNumber < 1:
            raise ValueError, "motorNumber must be between 1 and 4 inclusive"

        if motorNumber == 1:
            this.motorA = MOTOR1_A
            this.motorB = MOTOR1_B
            this.motorPwm = GPIO.PWM(MOTOR1_EN, PWM_FREQ)

        elif motorNumber == 2:
            this.motorA = MOTOR2_A
            this.motorB = MOTOR2_B
            this.motorPwm = MOTOR2_EN
            this.motorPwm = GPIO.PWM(MOTOR2_EN, PWM_FREQ)

        elif motorNumber == 3:
            this.motorA = MOTOR3_A
            this.motorB = MOTOR3_B
            this.motorPwm = MOTOR3_EN
            this.motorPwm = GPIO.PWM(MOTOR3_EN, PWM_FREQ)

        elif motorNumber == 4:
            this.motorA = MOTOR4_A
            this.motorB = MOTOR4_B
            this.motorPwm = GPIO.PWM(MOTOR4_EN, PWM_FREQ)

        # Initialize motor driver to idle state
        this.motorPwm.start(0)
        this.setDirection(RELEASE)
        latch_tx()

    def latch_tx():
        GPIO.output(MOTORLATCH, LOW)
        GPIO.output(MOTORDATA, GPIO.LOW)

        for i in range(0,7):
            GPIO.output(MOTORCLK, LOW)
            if (latchState >> i) & 0x01 == 0x01:
                GPIO.output(MOTORDATA, HIGH)
            else:
                GPIO.output(MOTORDATA, LOW)
            GPIO.output(MOTORCLK, HIGH)

        GPIO.output(MOTORLATCH, HIGH)

    def setDirection(direction):
        if direction == FORWARD:
            this.latchState |= 1 << this.motorA
            this.latchState &= !(1 << this.motorB)
        if direction == BACKWARD:
            this.latchState &= !(1 << this.motorA)
            this.latchState |= 1 << this.motorB
        if direction == RELEASE:
            this.latchState &= !(1 << this.motorA)
            this.latchState &= !(1 << this.motorB)

        latch_tx()

    def setSpeed(percentSpeed):
        this.motorPwm.ChangeDutyCycle(percentSpeed)

