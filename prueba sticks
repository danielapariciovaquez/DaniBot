import pygame
import time
import sys

# ---------------------------------
# Inicialización de pygame
# ---------------------------------
pygame.init()
pygame.joystick.init()

# ---------------------------------
# Comprobación de mandos
# ---------------------------------
num_joysticks = pygame.joystick.get_count()

if num_joysticks == 0:
    print("ERROR: No se detecta ningún mando")
    sys.exit(1)

js = pygame.joystick.Joystick(0)
js.init()

print("Mando detectado:")
print("  Nombre :", js.get_name())
print("  Ejes   :", js.get_numaxes())
print("  Botones:", js.get_numbuttons())
print("  Hats   :", js.get_numhats())
print("-" * 40)

# ---------------------------------
# Bucle principal
# ---------------------------------
try:
    while True:
        pygame.event.pump()

        # ---------
        # Ejes
        # ---------
        axes = []
        for i in range(js.get_numaxes()):
            val = js.get_axis(i)
            axes.append(f"A{i}:{val:+.3f}")

        # ---------
        # Botones
        # ---------
        buttons = []
        for i in range(js.get_numbuttons()):
            val = js.get_button(i)
            buttons.append(f"B{i}:{val}")

        # ---------
        # Hats (cruceta)
        # ---------
        hats = []
        for i in range(js.get_numhats()):
            val = js.get_hat(i)
            hats.append(f"H{i}:{val}")

        # Limpia pantalla
        print("\033[H\033[J", end="")

        print("EJES:")
        print("  " + "  ".join(axes))
        print()
        print("BOTONES:")
        print("  " + "  ".join(buttons))
        print()
        print("CRUCETA:")
        print("  " + "  ".join(hats))

        time.sleep(0.05)  # 20 Hz

except KeyboardInterrupt:
    print("\nSalida por Ctrl+C")

finally:
    pygame.quit()
