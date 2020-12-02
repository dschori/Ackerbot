import RPi.GPIO as GPIO
import time
import numpy as np

# Pin Definitons:
led_pin = 12  # Board pin 12
but_pin = 18  # Board pin 18

def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    GPIO.setup(led_pin, GPIO.OUT)  # LED pin set as output
    GPIO.setup(but_pin, GPIO.IN)  # button pin set as input

    # Initial state for LEDs:
    GPIO.output(led_pin, GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    try:
        while True:
            times = []
            for i in range(4):
                while GPIO.input(but_pin) == 0:
                    pass
                time_now = time.time()
                while GPIO.input(but_pin) == 1:
                    pass
                time_used = ((time.time()-time_now)-0.0015)*2000.
                time_used = np.min((time_used, 1.0))
                time_used = np.max((time_used, -1.0))
                times.append(time_used)

            print(np.mean(times))
            time.sleep(0.05)

    finally:
        GPIO.cleanup()  # cleanup all GPIOs

if __name__ == '__main__':
    main()