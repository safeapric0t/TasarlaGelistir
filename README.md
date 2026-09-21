# ProBot Firmware

ESP32 tabanlı ProBot kontrol yazılımı. Firmware; PCA9685 üzerinden motor/servo çıkışlarını, BNO08x IMU üzerinden yön bilgisini ve joystick üzerinden manuel sürüşü yönetir.

Kodun temel yapısı iki ana çalışma moduna ayrılır:

* **Teleop:** Joystick üzerinden manuel kontrol
* **Autonomous:** Önceden tanımlanmış zaman tabanlı hareket sekansı

---

## Hardware

### Controller

Ana kontrolcü olarak ESP32/ProBot kartı kullanılmaktadır.

I2C hattı:

| Sinyal | GPIO |
| ------ | ---: |
| SDA    |    8 |
| SCL    |    9 |

I2C frekansı:

```cpp
Wire.setClock(400000);
```

### PCA9685

Motor ve servo çıkışları PCA9685 üzerinden kontrol edilir.

```cpp
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
```

PWM frekansı:

```cpp
const int PWM_FREQ = 50;
```

Kanal dağılımı:

| Kanal | Kullanım          |
| ----: | ----------------- |
|     0 | Front Left Motor  |
|     1 | Front Right Motor |
|     2 | Rear Left Motor   |
|     3 | Rear Right Motor  |
|     4 | Intake            |
|     7 | Lift              |
|     8 | Servo             |
|    14 | Climb 1           |
|    15 | Climb 2           |

Bu kanal tanımları kod içerisinde sabit olarak tutulmaktadır.

---

## Motor Control

Motor çıkışları PCA9685 PWM sinyali üzerinden kontrol edilir.

Standart motor kontrolünde merkez değer:

```cpp
STOP_TICK = 307
```

ileri ve geri sınırlar:

```cpp
MAX_FWD = 410
MAX_REV = 205
```

Motor gücü `-1.0` ile `1.0` arasında alınarak PWM değerine çevrilir:

```cpp
pulse = STOP_TICK + (power * 102);
```

ve ardından sınırlandırılır:

```cpp
pulse = constrain(pulse, MAX_REV, MAX_FWD);
```

`power` değeri yaklaşık `±0.12` bölgesindeyse motor stop konumuna gönderilir.

### Intake

Intake ayrı bir stop değeri kullanır:

```cpp
const int Intake_stop_tick = 363;
```

Bu nedenle intake için ayrı bir PWM fonksiyonu bulunmaktadır:

```cpp
writeIntake()
```

### Climb

Climb motorları standart `writeSpark()` fonksiyonu yerine doğrudan PCA9685 çıkışı ile kontrol edilir.

```cpp
CLIMB1 = 14
CLIMB2 = 15
```

İki motor ters yönlerde çalıştırılarak mekanizmanın hareket yönü belirlenir.

---

## Servo Control

Servo, PCA9685 kanal `8` üzerinden kontrol edilir.

Servo çalışma aralığı:

```text
0° - 270°
```

Derece → PWM dönüşümü:

```cpp
pulse = map(degree, 0, 270, 102, 614);
```

Servo doğrudan hedef pozisyona gönderilmek yerine:

```cpp
servoCurrentPos
servoTargetPos
```

değişkenleri üzerinden kademeli olarak hareket ettirilir.

Hareket hızı:

```cpp
SERVO_SPEED = 8.0f;
```

## Bu yapı `updateServoMotion()` içerisinde uygulanmaktadır.

# BNO08x IMU

BNO08x, I2C üzerinden kullanılmaktadır.

Başlatma sırasında iki olası I2C adresi denenir:

```cpp
0x4A
0x4B
```

```cpp
bno08x.begin_I2C(0x4A, &Wire)
bno08x.begin_I2C(0x4B, &Wire)
```

Sensor report:

```cpp
SH2_ARVR_STABILIZED_RV
```

olarak ayarlanır.

## Yaw Calculation

BNO08x quaternion verisi:

```text
qw
qx
qy
qz
```

olarak alınır.

Yaw:

```cpp
atan2(
    2.0f * (qw * qz + qx * qy),
    1.0f - 2.0f * (qy * qy + qz * qz)
);
```

ile hesaplanır ve dereceye çevrilir.

Sonuç:

```cpp
currentYaw
```

değişkeninde tutulur.

---

# Drive Control

## Mecanum Drive

Teleop modunda joystick'ten üç eksen okunur:

```cpp
float x = js.getRawAxis(2);
float y = -js.getRawAxis(1);
float z = js.getRawAxis(0);
```

Bunlar dört motor için mecanum kinematiğine dönüştürülür:

```cpp
tFL = y + x + z;
tFR = y - x - z;
tRL = y - x + z;
tRR = y + x - z;
```

Motor değerleri maksimum mutlak değer `1.0` olacak şekilde normalize edilir.

```cpp
maxP = max({
    1.0f,
    fabs(tFL),
    fabs(tFR),
    fabs(tRL),
    fabs(tRR)
});
```

Sonrasında motorlara gönderilir:

```cpp
writeSpark(M_FL, tFL / maxP);
writeSpark(M_FR, -(tFR / maxP));
writeSpark(M_RL, tRL / maxP);
writeSpark(M_RR, -(tRR / maxP));
```

Sağ taraftaki motorların yönü yazılım içerisinde terslenmiştir.

---

# Arcade Drive

Autonomous içerisinde mecanum kontrolü yerine arcade drive fonksiyonları kullanılmaktadır.

```cpp
driveManualArcade(forwardPower, turnPower)
```

Sol ve sağ taraf için:

```cpp
left  = forwardPower + turnPower;
right = forwardPower - turnPower;
```

hesaplanır.

Daha sonra motor yönlerine göre gerekli inversion uygulanır.

---

# Gyro Assisted Straight Drive

Kodda IMU kullanarak düz gitme fonksiyonu da bulunmaktadır:

```cpp
driveStraight(power, targetYaw)
```

Öncelikle mevcut yaw ile hedef yaw arasındaki açı farkı hesaplanır:

```cpp
error = normalizeAngle(targetYaw - currentYaw);
```

Daha sonra basit P kontrol uygulanır:

```cpp
correction = error * 0.02f;
```

Correction:

```cpp
-0.3 ... +0.3
```

aralığında sınırlandırılır.

Sol ve sağ motor güçleri:

```cpp
leftPower  = power - correction;
rightPower = power + correction;
```

şeklinde ayarlanır.

Bu sayede robot hedef heading'i korumaya çalışır.

> Autonomous kodunun mevcut halinde `driveStraight()` kullanılmıyor; mevcut autonomous hareketleri `driveManualArcade()` ve `turnInPlace()` üzerinden gerçekleştiriliyor.

---

# Autonomous State Machine

Autonomous sistemi `autoStep` değişkeni ile çalışan basit bir state machine'dir.

```cpp
int autoStep = 0;
unsigned long autoTimer = 0;
```

Her state içerisinde `millis()` kullanılarak süre kontrol edilir.

Temel akış:

```text
State 0
  ↓
Initialize
  ↓
State 1
  Forward
  ↓
State 2
  Stop
  ↓
State 3
  Turn
  ↓
State 4
  Forward
  ↓
State 5
  Stop
  ↓
State 6
  Turn
  ↓
State 7
  Forward + Servo
  ↓
State 8
  Lift
  ↓
State 9
  Stop
```

Örneğin dönüş state'i:

```cpp
case 3:
    if (millis() - autoTimer < AUTO_TURN_MS) {
        turnInPlace(AUTO_TURN_POWER);
    }
```

şeklinde çalışır.

Autonomous'ta kullanılan temel parametreler:

```cpp
AUTO_FWD_POWER
AUTO_TURN_POWER
AUTO_TURN_TOLERANCE
AUTO_FWD_MS
AUTO_STOP_MS
AUTO_TURN_MS
AUTO_LIFT_MS
```

---

# Mirrored Autonomous

Kod içerisinde autonomous rotasının ters yönde çalıştırılabilmesi için ikinci bir autonomous fonksiyonu da bulunmaktadır:

```cpp
autonomousLoopMirrored()
```

Bu fonksiyon mevcut kodda yorum satırları içerisindedir.

Normal dönüş:

```cpp
turnInPlace(AUTO_TURN_POWER);
```

Mirror dönüş:

```cpp
turnInPlace(-AUTO_TURN_POWER);
```

şeklindedir.

---

# Robot Lifecycle

Firmware içerisinde temel robot yaşam döngüsü:

```text
robotInit()
    ↓
teleopInit() / autonomousInit()
    ↓
teleopLoop() / autonomousLoop()
    ↓
robotEnd()
```

## `robotInit()`

Başlangıçta:

1. Serial haberleşme başlatılır.
2. I2C başlatılır.
3. BNO08x aranır.
4. IMU report aktif edilir.
5. PCA9685 başlatılır.
6. PWM frekansı ayarlanır.
7. Climb çıkışları kapatılır.
8. Servo başlangıç pozisyonuna alınır.

## `teleopInit()`

Teleop başlamadan önce:

```cpp
stopMotors();
writeIntake(0.0f);
```

ile sürüş motorları ve intake durdurulur.

## `autonomousInit()`

Autonomous başlangıcında motorlar ve mekanizmalar sıfırlanır ve state machine:

```cpp
autoStep = 0;
```

ile başlangıç state'ine alınır.

---

# Telemetry

Teleop sırasında her 200 ms'de gyro bilgisi güncellenir ve telemetry olarak gönderilir.

```cpp
publishTelemetry();
```

Gönderilen değerler:

```text
Gyro yaw
Gyro ready
Gyro age
Update count
Sensor ID
```

Bu yapı özellikle IMU'nun çalışıp çalışmadığını ve sensör verisinin ne kadar güncel olduğunu kontrol etmek için kullanılmaktadır.

---

# Safety / Stop Behavior

Motorları durdurmak için merkezi bir fonksiyon kullanılır:

```cpp
void stopMotors()
```

Bu fonksiyon dört drive motorunun tamamını `0` güç değerine gönderir.

Motor kontrol fonksiyonlarında ayrıca düşük güç değerleri stop konumuna çekilir. Böylece joystick'in merkezindeki küçük değerlerin motorları hareket ettirmesi engellenir.

---

# Code Structure

Firmware içerisinde kontrol mantığı temel olarak şu fonksiyonlara ayrılmıştır:

```text
robotInit()
│
├── I2C / BNO08x initialization
├── PCA9685 initialization
└── Servo initialization

teleopLoop()
│
├── Telemetry
├── Joystick input
├── Mecanum kinematics
├── Intake
├── Lift
├── Climb
└── Servo

autonomousLoop()
│
├── Forward
├── Stop
├── Turn
├── Forward
├── Turn
├── Servo
└── Lift

Sensor / Control
│
├── updateGyro()
├── driveStraight()
├── driveManualArcade()
├── turnInPlace()
└── normalizeAngle()

Output
│
├── writeSpark()
├── writeIntake()
├── setSingleMotorI2C()
├── writeServo()
└── writeServoDeg()
```

Bu ayrım sayesinde donanım çıkışları ile robot hareket algoritmaları ayrı fonksiyonlarda tutulmaktadır.

---

## Dependencies

```cpp
#include <probot.h>
#include <probot/io/joystick_api.hpp>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
```

Temel harici bileşenler:

* ProBot API
* ProBot Joystick API
* Wire / I2C
* Adafruit PWM Servo Driver
* Adafruit BNO08x
