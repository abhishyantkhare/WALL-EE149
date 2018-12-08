# Ultrasonic Sensor Raspberry Pi Class

import RPi.GPIO as GPIO
import time

class UltraSonicSensor(object):

    def __init__(self):
        # Sets up the UltrasonicSensor GPIO pins and board.
        pass        

    def setup(trigger_pin = 7, echo_pin = 11):
        GPIO.setmode(GPIO.BOARD)
        self.PIN_TRIGGER = trigger_pin
        self.PIN_ECHO = echo_pin

        GPIO.setup(self.PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(self.PIN_ECHO, GPIO.IN)
        GPIO.output(self.PIN_TRIGGER, GPIO.LOW)
        print("Waiting for sensor to settle")
        time.sleep(2)

    def get_distance(self):
        print("Calculating distance")
        GPIO.output(PIN_TRIGGER, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        while GPIO.input(PIN_ECHO) == 0:
            pulse_start_time = time.time()
        while GPIO.input(PIN_ECHO) == 1:
            pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)
        print("Distance: ", distance, " cm")
        return distance
            
    def destroy(self):
        GPIO.cleanup()

# Small "Unit Test"
ultraSonicSensor = UltraSonicSensor()
ultraSonicSensor.setup()
for i in range(20):
    ultraSonicSensor.get_distance()
ultraSonicSensor.destroy()
