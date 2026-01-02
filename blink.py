import pigpio
import time

PIN_24 = 24
PIN_25 = 25

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiod no está activo")

# Frecuencia PWM (Hz)
pi.set_PWM_frequency(PIN_24, 800)
pi.set_PWM_frequency(PIN_25, 800)

try:
    while True:
        # Fade in
        for duty in range(0, 256):
            pi.set_PWM_dutycycle(PIN_24, duty)
            pi.set_PWM_dutycycle(PIN_25, duty)
            time.sleep(0.01)

        # Fade out
        for duty in range(255, -1, -1):
            pi.set_PWM_dutycycle(PIN_24, duty)
            pi.set_PWM_dutycycle(PIN_25, duty)
            time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    pi.set_PWM_dutycycle(PIN_24, 0)
    pi.set_PWM_dutycycle(PIN_25, 0)
    pi.stop()
