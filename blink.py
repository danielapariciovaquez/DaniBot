import RPi.GPIO as GPIO
import time

# Usar numeración BCM
GPIO.setmode(GPIO.BCM)

# Definición de pines
PIN_GPIO_3 = 23
PIN_GPIO_15 = 24

# Configuración como salidas
GPIO.setup(PIN_GPIO_3, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_GPIO_15, GPIO.OUT, initial=GPIO.LOW)

try:
    while True:
        # Encender ambos pines
        GPIO.output(PIN_GPIO_3, GPIO.HIGH)
        GPIO.output(PIN_GPIO_15, GPIO.HIGH)
        time.sleep(1.0)

        # Apagar ambos pines
        GPIO.output(PIN_GPIO_3, GPIO.LOW)
        GPIO.output(PIN_GPIO_15, GPIO.LOW)
        time.sleep(1.0)

except KeyboardInterrupt:
    pass

finally:
    # Apagar pines y liberar GPIO
    GPIO.output(PIN_GPIO_3, GPIO.LOW)
    GPIO.output(PIN_GPIO_15, GPIO.LOW)
    GPIO.cleanup()
