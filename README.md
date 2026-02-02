# 🛡️ Novad - FTC Defense Library# Novad - FTC Defense Library



**Stop Getting Pushed Around.****Stop Getting Pushed Around.**



Novad is a defense library for FTC robots that automatically resists being pushed by opponents. With **Predictive Defense**, Novad responds faster than any other defense library—without sacrificing accuracy.Novad is a defense library for FTC robots that prevents opponents from pushing your robot around during matches. Using odometry to detect unwanted movement, Novad automatically applies counter-force to resist defense.



## ✨ What Makes Novad Different## Features



### 🚀 Predictive Defense (NEW!)- 🛡️ **Push Resistance** - Automatically resist when opponents push your robot

- 🎯 **Position Lock** - Lock your robot in place for maximum resistance

Traditional defense libraries react AFTER you've been pushed. **Novad predicts WHERE you'll be pushed and counters BEFORE you get there.**- 🎮 **Driver Override** - Instantly disables when driver touches joysticks

- ⚡ **Smooth Ramp-Up** - Gradual power increase prevents jerky movements

```- 🔧 **Easy Tuning** - Real-time PID tuning with panels.bylazar.com

Traditional Defense:        Novad Predictive Defense:- 🔌 **Universal** - Works with Pedro Pathing, RoadRunner, or any odometry

                           

Push → Movement → Detect → │ Push → Accel Detect → Predict → Counter## Quick Start

                  Counter  │                       

                           │ (Counter happens BEFORE movement!)### 1. Add Dependency

```

```gradle

**How it works:**// In build.gradle

1. Monitors acceleration (sudden change = push detected)repositories {

2. Predicts position X milliseconds in the future    maven { url 'https://jitpack.io' }

3. Applies counter-force immediately}

4. Result: **50ms+ faster response** without sacrificing accuracy

dependencies {

## 🎯 Features    implementation 'com.github.novad:novad:1.0.0'

}

- 🚀 **Predictive Defense** - Counter pushes before they happen```

- 🛡️ **Push Resistance** - Automatically resist when opponents push your robot

- 🔒 **Position Lock** - Lock your robot in place for maximum resistance### 2. Configure Your Robot

- 🎮 **Driver Override** - Instantly disables when driver touches joysticks

- ⚡ **Instant Boost** - Multiplies response when impact detectedEdit `NovadConstants.java`:

- 📊 **FTC Dashboard** - Live tune ALL values in real-time

- 📍 **Pinpoint Support** - Works with GoBilda Pinpoint odometry```java

public static final DriveType DRIVE_TYPE = DriveType.MECANUM_4_MOTOR;

## 📦 Installationpublic static final OdometryType ODOMETRY_TYPE = OdometryType.THREE_WHEEL;

```

1. **Clone or download this repo**

2. **Copy the `novad` folder** to your TeamCode:### 3. Use in TeleOp

   ```

   TeamCode/src/main/java/org/firstinspires/ftc/teamcode/novad/```java

   ```// Initialize

3. **Copy `NovadConstants.java` and `NovadTeleOp.java`** to TeamCode rootNovad novad = new Novad(odometry, drivetrain);



## ⚙️ Configuration// In your loop - that's it!

novad.defense(

Edit `NovadConstants.java` - it's the **only file you need to configure!**    gamepad1.left_stick_x,

    -gamepad1.left_stick_y,

```java    gamepad1.right_stick_x

// Motor names (from your Robot Configuration));

public static String LEFT_FRONT_MOTOR = "frontLeft";```

public static String LEFT_REAR_MOTOR = "backLeft";

public static String RIGHT_FRONT_MOTOR = "frontRight";## Documentation

public static String RIGHT_REAR_MOTOR = "backRight";

Visit [novad.dev](https://novad.dev) for complete documentation, tuning guides, and examples.

// Motor directions

public static boolean LEFT_FRONT_REVERSED = true;## Tuning

public static boolean LEFT_REAR_REVERSED = true;

public static boolean RIGHT_FRONT_REVERSED = false;1. Run the `DefenseTuning` OpMode

public static boolean RIGHT_REAR_REVERSED = false;2. Open [panels.bylazar.com](https://panels.bylazar.com) (or `192.168.43.1:8080/dash`)

3. Have a partner push your robot

// Odometry type4. Adjust PID sliders until robot resists stably

public static OdometryType ODOMETRY_TYPE = OdometryType.PINPOINT;5. Copy values to `NovadConstants.java`



// Pinpoint config (if using)## Integration Examples

public static String PINPOINT_DEVICE_NAME = "pinpoint";

```### With Pedro Pathing



All PIDF values are **tunable live via FTC Dashboard**!```java

Follower follower = new Follower(hardwareMap);

## 🎮 UsageNovadOdometry odometry = new PedroPathingAdapter(follower);

Novad novad = new Novad(odometry, drivetrain);

Just **one line** in your TeleOp loop:```



```java### With RoadRunner

novad.defense(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

``````java

SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

### Complete ExampleNovadOdometry odometry = new RoadRunnerAdapter(drive);

Novad novad = new Novad(odometry, drivetrain);

See `NovadTeleOp.java` for a working example with:```

- Automatic odometry selection (Pinpoint or Three-Wheel)

- Live PIDF tuning from Dashboard### Position Lock

- Position lock toggle

- Defense enable/disable```java

// Lock position when A pressed

## 🔧 Tuningif (gamepad1.a) {

    novad.lockPosition();

1. **Deploy** `NovadTeleOp.java` to your robot}

2. **Connect** to FTC Dashboard:

   - `http://192.168.43.1:8080/dash` (on robot WiFi)// Unlock when B pressed

   - OR [panels.bylazar.com](https://panels.bylazar.com)if (gamepad1.b) {

3. **Find** `NovadConstants` in the Configuration panel    novad.unlockPosition();

4. **Adjust** values and see changes in real-time!}

```

### Recommended Starting Values

## License

| Parameter | Value | Description |

|-----------|-------|-------------|MIT License - see [LICENSE](LICENSE) for details.

| `TRANS_P` | 0.046 | Position hold strength |
| `TRANS_D` | 0.01  | Position damping |
| `HEADING_P` | 0.67 | Rotation hold strength |
| `HEADING_D` | 0.006 | Rotation damping |
| `PREDICTION_LOOKAHEAD_MS` | 50 | How far ahead to predict |
| `INSTANT_BOOST_MULTIPLIER` | 1.5 | Response boost on impact |

## 📁 File Structure

```
TeamCode/
├── NovadConstants.java     ← Configure everything here!
├── NovadTeleOp.java        ← Example TeleOp (copy and customize)
└── novad/
    ├── Novad.java          ← Main API
    ├── adapters/
    │   ├── PinpointOdometry.java
    │   ├── ThreeWheelOdometry.java
    │   └── MecanumDriveAdapter.java
    ├── config/
    │   ├── NovadConfig.java
    │   ├── PIDFCoefficients.java
    │   └── MotorDirection.java
    ├── controller/
    │   └── DefenseController.java
    ├── interfaces/
    │   ├── NovadOdometry.java
    │   └── NovadDrivetrain.java
    └── util/
        ├── Vector2D.java
        ├── PIDController.java
        └── MathUtils.java
```

## 🤖 Supported Hardware

**Odometry:**
- ✅ GoBilda Pinpoint (recommended)
- ✅ Three-wheel dead wheel
- ✅ Drive motor encoders (less accurate)

**Drivetrain:**
- ✅ Mecanum 4-wheel

## 📚 Documentation

Visit the [Novad Website](docs/website/index.html) for full documentation, tuning guides, and API reference.

## 📄 License

MIT License - Free for all FTC teams to use and modify.

---

**Built for FTC teams, by FTC teams. 🤖**

*Stop getting pushed around. Start using Novad.*
