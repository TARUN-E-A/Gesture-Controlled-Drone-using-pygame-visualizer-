# Gesture Controlled Drone Controller

 Gesture Controlled Drone System

---

## Aim

To design and implement a gesture-based drone control system using ESP32, MPU6050 sensor, and LDR, where hand movements control pitch and roll while light intensity controls throttle, with real-time visualization and programmable drone integration.

---

## 🧰 Hardware Required

- ESP32 Development Board  
- MPU6050 Accelerometer & Gyroscope Module  
- LDR (Light Dependent Resistor)  
- 10kΩ Resistor  
- LiteWing Programmable Drone  
- Jumper Wires  
- Breadboard  

---

## 💻 Software Required

- Arduino IDE  
- Python 3.x  
- Crazyflie cflib Python SDK  
- PySerial  
- Pygame  

---

## 🔌 Connections

### MPU6050 to ESP32

| MPU6050 | ESP32 |
|--------|------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

### LDR Connection

- One terminal of LDR → 3.3V  
- Other terminal of LDR → GPIO 34  
- 10kΩ resistor from GPIO 34 → GND

### Gesture controller img:

<img width="382" height="502" alt="image" src="https://github.com/user-attachments/assets/18357d24-cf1f-4711-ac31-a1836ed7462f" />

### 🎮 Gesture Mapping

<img width="707" height="352" alt="image" src="https://github.com/user-attachments/assets/eb0d0e0b-11f6-413c-9ab2-2509c0b5062f" />

## ESP32 Gesture Controller Code:
```
#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
MPU6050 mpu;

// LDR pin
const int LDR_PIN = 34;

// MPU raw
int16_t ax, ay, az, gx, gy, gz;

// Control values
int pitch = 0, roll = 0, throttle = 0;

// Calibration offsets
int pitch_offset = 0;
int roll_offset  = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  SerialBT.begin("ESP32_GESTURE");

  pinMode(LDR_PIN, INPUT);

  mpu.initialize();
  Serial.println("ESP32 started");

  // -------- CALIBRATION --------
  Serial.println("Calibrating... Keep hand steady");

  long p_sum = 0, r_sum = 0;

  for (int i = 0; i < 200; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int p = map(ay, -12000, 12000, -30, 30);
    int r = map(ax, -12000, 12000, -30, 30);

    p_sum += p;
    r_sum += r;

    delay(5);
  }

  pitch_offset = p_sum / 200;
  roll_offset  = r_sum / 200;

  Serial.print("Pitch offset = ");
  Serial.println(pitch_offset);
  Serial.print("Roll offset  = ");
  Serial.println(roll_offset);

  Serial.println("Calibration done");
}

void loop() {
  // Read MPU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // -------- PITCH & ROLL --------
  pitch = map(ay, -12000, 12000, -30, 30) - pitch_offset;
  roll  = map(ax, -12000, 12000, -30, 30) - roll_offset;

  pitch = constrain(pitch, -30, 30);
  roll  = constrain(roll,  -30, 30);

  // -------- THROTTLE (LDR) --------
  int ldrValue = analogRead(LDR_PIN);
  ldrValue = constrain(ldrValue, 0, 4095);

  throttle = map(ldrValue, 800, 3000, 0, 100);
  throttle = constrain(throttle, 0, 100);

  // -------- SEND: THR,PIT,ROL --------
  SerialBT.print(throttle);
  SerialBT.print(",");
  SerialBT.print(pitch);
  SerialBT.print(",");
  SerialBT.println(roll);

  // USB debug (optional)
  Serial.print("THR=");
  Serial.print(throttle);
  Serial.print(" PIT=");
  Serial.print(pitch);
  Serial.print(" ROL=");
  Serial.println(roll);

  delay(40); // ~25 Hz
}

```

### Python Visualizer Code:

```
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

```
### Output:
<img src="https://github.com/user-attachments/assets/dee9f5c6-88e3-4ef6-88a7-e5c750f5e100" width="300" height="400" alt="WhatsApp Image">


VIDEO DRIVE LINK:

https://drive.google.com/drive/u/0/folders/1rLTd-d-tOQheCXWnEsoRKPhyxdBzVzgB 

### Result:

The system successfully converts hand gestures into drone control signals.
Pitch and roll are controlled through hand tilting, while throttle is controlled using light intensity via the LDR sensor. The commands are transmitted wirelessly and visualized in real time, and can be sent to a programmable drone.



