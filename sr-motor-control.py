import Rpi.GPIO as GPIO
import Rpi.GPIO.LOW as LOW
import Rpi.GPIO.HIGH as HIGH

class ShiftRegisterMotorControl:
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

    # Arduino pin names for interface to 74HCT595 latch
    MOTORLATCH = 12 # TOOD
    MOTORCLK = 4 # TOOD
    MOTORENABLE = 7 # TOOD
    MOTORDATA = 8 # TOOD

    def __init__(this, motorNumber):
        this.motorNumber = motorNumber

    def latch_tx(value):
        GPIO.output(MOTORLATCH, LOW)
        GPIO.output(MOTORDATA, GPIO.LOW)

        for i in range(0,7):
            GPIO.output(MOTORCLK, LOW)
            if (value >> i) & 0x01 == 0x01:
                GPIO.output(MOTORDATA, HIGH)
            else:
                GPIO.output(MOTORDATA, LOW)
            GPIO.output(MOTORCLK, HIGH)

        GPIO.output(MOTORLATCH, HIGH)

    def run(control):



    def setSpeed(control):
        # TODO: Set corresponding pin pwm rate

