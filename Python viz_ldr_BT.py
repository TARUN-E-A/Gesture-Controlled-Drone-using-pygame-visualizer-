import serial
import pygame
import sys
import time

# ================= BLUETOOTH =================
BT_PORT = "COM10"      # 🔴 your working Bluetooth COM port
BAUDRATE = 9600

print("Opening Bluetooth...")
ser = serial.Serial(BT_PORT, BAUDRATE, timeout=0.05)
time.sleep(2)
print("Bluetooth connected")

# ================= PYGAME =================
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Gesture Drone Simulator (Bluetooth + LDR)")

clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 28)
big_font = pygame.font.SysFont(None, 42)

# ================= DRONE STATE =================
x, y = WIDTH // 2, HEIGHT // 2

pitch = 0
roll = 0
throttle = 0

smooth_pitch = 0
smooth_roll = 0
alpha = 0.15

# ================= EMERGENCY STOP =================
emergency_stop = False

# ================= MAIN LOOP =================
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                emergency_stop = True
                throttle = 0
                smooth_pitch = 0
                smooth_roll = 0

    # -------- READ BLUETOOTH --------
    if not emergency_stop:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                # ESP32 sends: THR,PIT,ROL
                throttle, pitch, roll = map(int, line.split(","))

        except:
            pass
    else:
        pitch = 0
        roll = 0
        throttle = 0

    # -------- DEAD ZONE --------
    if abs(pitch) < 2: pitch = 0
    if abs(roll) < 2: roll = 0

    # -------- SMOOTHING --------
    smooth_pitch += alpha * (pitch - smooth_pitch)
    smooth_roll  += alpha * (roll  - smooth_roll)

    # -------- MOVEMENT --------
    if not emergency_stop:
        x += smooth_roll * 0.5
        y -= smooth_pitch * 0.5

    x = max(50, min(WIDTH - 50, x))
    y = max(50, min(HEIGHT - 50, y))

    # -------- DRAW --------
    screen.fill((10, 10, 20))

    # Drone body (size depends on throttle)
    radius = int(20 + throttle * 0.2)
    pygame.draw.circle(screen, (0, 255, 120), (int(x), int(y)), radius, 2)

    # Direction indicator
    pygame.draw.line(
        screen,
        (0, 200, 255),
        (int(x), int(y)),
        (int(x), int(y - smooth_pitch * 2)),
        2
    )

    # HUD
    hud = font.render(
        f"THR={throttle}  PIT={int(smooth_pitch)}  ROL={int(smooth_roll)}",
        True,
        (255, 255, 0)
    )
    screen.blit(hud, (20, 20))

    help_txt = font.render(
        "LDR = Throttle   SPACE = EMERGENCY STOP",
        True,
        (180, 180, 180)
    )
    screen.blit(help_txt, (20, 50))

    # -------- EMERGENCY WARNING --------
    if emergency_stop:
        warn = big_font.render(
            "EMERGENCY STOP ACTIVE",
            True,
            (255, 50, 50)
        )
        screen.blit(
            warn,
            (WIDTH // 2 - warn.get_width() // 2, HEIGHT // 2 - 20)
        )

    pygame.display.update()
    clock.tick(60)
