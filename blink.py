import RPi.GPIO as GPIO
import time

# ==============================
# CONFIGURACIÓN
# ==============================
GPIO.setmode(GPIO.BCM)

PIN_24 = 24
PIN_25 = 25

PWM_FREQ = 500        # Hz
FADE_STEP = 1         # incremento de duty (%)
FADE_DELAY = 0.1     # segundos entre pasos

# ==============================
# INICIALIZACIÓN GPIO
# ==============================
GPIO.setup(PIN_24, GPIO.OUT)
GPIO.setup(PIN_25, GPIO.OUT)

pwm24 = GPIO.PWM(PIN_24, PWM_FREQ)
pwm25 = GPIO.PWM(PIN_25, PWM_FREQ)

pwm24.start(0)
pwm25.start(0)

try:
    while True:
        # --------------------------
        # FADE IN
        # --------------------------
        for duty in range(0, 101, FADE_STEP):
            pwm24.ChangeDutyCycle(duty)
            pwm25.ChangeDutyCycle(duty)
            time.sleep(FADE_DELAY)

        # --------------------------
        # FADE OUT
        # --------------------------
        for duty in range(100, -1, -FADE_STEP):
            pwm24.ChangeDutyCycle(duty)
            pwm25.ChangeDutyCycle(duty)
            time.sleep(FADE_DELAY)

except KeyboardInterrupt:
    pass

finally:
    pwm24.stop()
    pwm25.stop()
    GPIO.cleanup()
