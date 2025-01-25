import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# Define pins
servo_pin = 23
sensor = 26
TRIG = 11
ECHO = 12
Relay_PIN = 13
Green_LED = 15
Red_LED = 16
Button_PIN = 8
MotorControl_PIN = 32

# Setup GPIO pins
GPIO.setup(Button_PIN, GPIO.IN)
GPIO.setup(MotorControl_PIN, GPIO.OUT)
GPIO.output(MotorControl_PIN, False)

GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)

GPIO.setup(sensor, GPIO.IN)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(Relay_PIN, GPIO.OUT)
GPIO.setup(Green_LED, GPIO.OUT)
GPIO.setup(Red_LED, GPIO.OUT)

# Initial states
GPIO.output(Relay_PIN, False)
GPIO.output(Green_LED, False)
GPIO.output(Red_LED, False)
GPIO.output(TRIG, False)

# Start servo PWM
pwm.start(0)

print("Waiting for sensor to settle...")
time.sleep(1)

def set_angle(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        button_state = GPIO.input(Button_PIN)
        print("Button State:", button_state)

        if button_state == True:
            GPIO.output(MotorControl_PIN, True)  # Turn on motor (e.g., a pump)
            while GPIO.input(Button_PIN) == True:  # Wait for button to be released
                if GPIO.input(sensor):  # Fire detection sensor
                    print("Fire not detected")
                    time.sleep(0.2)
                else:
                    print("No fire detected, proceeding with actions.")
                    time.sleep(0.2)
                    set_angle(180)
                    time.sleep(1)
                    set_angle(0)
                    time.sleep(1)
                    
                    # Trigger ultrasonic sensor to measure water level
                    GPIO.output(TRIG, True)
                    time.sleep(0.00001)
                    GPIO.output(TRIG, False)

                    # Record the time when echo pin is low
                    while GPIO.input(ECHO) == 0:
                        pulse_start = time.time()

                    # Record the time when echo pin goes high
                    while GPIO.input(ECHO) == 1:
                        pulse_end = time.time()

                    # Calculate the pulse duration
                    pulse_duration = pulse_end - pulse_start

                    # Calculate distance (speed of sound is 343 m/s, so distance = time * speed)
                    distance = pulse_duration * 17150  # in cm
                    distance = round(distance, 2)
                    print("Water Level (cm):", distance)

                    time.sleep(0.5)

                    # Check water level and control relay (motor)
                    if distance > 12:  # If water level is high enough, turn off the motor
                        GPIO.output(Relay_PIN, False)
                        GPIO.output(Green_LED, True)  # Green LED = motor off
                        GPIO.output(Red_LED, False)
                        print("Motor OFF")
                    else:  # If water level is low, turn on the motor
                        GPIO.output(Relay_PIN, True)
                        GPIO.output(Green_LED, False)
                        GPIO.output(Red_LED, True)  # Red LED = motor on
                        print("Motor ON")
                    time.sleep(0.2)
        else:
            GPIO.output(MotorControl_PIN, False)
            GPIO.output(Green_LED, False)
            GPIO.output(Red_LED, False)
            time.sleep(0.2)

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
