# 🛡️ Novad# 🛡️ Novad - FTC Defense Library# Novad - FTC Defense Library



**Stop getting pushed around.**



Novad automatically keeps your FTC robot in place when opponents try to push you.**Stop Getting Pushed Around.****Stop Getting Pushed Around.**



## Setup (5 minutes)



### Step 1: Copy FilesNovad is a defense library for FTC robots that automatically resists being pushed by opponents. With **Predictive Defense**, Novad responds faster than any other defense library—without sacrificing accuracy.Novad is a defense library for FTC robots that prevents opponents from pushing your robot around during matches. Using odometry to detect unwanted movement, Novad automatically applies counter-force to resist defense.

Copy the `novad` folder and these two files into your TeamCode:

```

TeamCode/src/main/java/org/firstinspires/ftc/teamcode/

├── NovadSetup.java     ← Configure your robot here## ✨ What Makes Novad Different## Features

├── NovadTeleOp.java    ← Ready-to-use TeleOp

└── novad/              ← The library (don't edit)

```

### 🚀 Predictive Defense (NEW!)- 🛡️ **Push Resistance** - Automatically resist when opponents push your robot

### Step 2: Open NovadSetup.java

- 🎯 **Position Lock** - Lock your robot in place for maximum resistance

Fill in YOUR motor names (copy from your Robot Configuration):

Traditional defense libraries react AFTER you've been pushed. **Novad predicts WHERE you'll be pushed and counters BEFORE you get there.**- 🎮 **Driver Override** - Instantly disables when driver touches joysticks

```java

public static String FRONT_LEFT  = "frontLeft";   // ← Your name here- ⚡ **Smooth Ramp-Up** - Gradual power increase prevents jerky movements

public static String FRONT_RIGHT = "frontRight";  // ← Your name here

public static String BACK_LEFT   = "backLeft";    // ← Your name here```- 🔧 **Easy Tuning** - Real-time PID tuning with panels.bylazar.com

public static String BACK_RIGHT  = "backRight";   // ← Your name here

```Traditional Defense:        Novad Predictive Defense:- 🔌 **Universal** - Works with Pedro Pathing, RoadRunner, or any odometry



### Step 3: Pick Your Odometry                           



**Using GoBilda Pinpoint?**Push → Movement → Detect → │ Push → Accel Detect → Predict → Counter## Quick Start

```java

public static boolean USE_PINPOINT = true;                  Counter  │                       

public static String PINPOINT_NAME = "pinpoint";  // Name in Robot Config

```                           │ (Counter happens BEFORE movement!)### 1. Add Dependency



**Using Three Dead Wheels?**```

```java

public static boolean USE_PINPOINT = false;```gradle

public static boolean USE_THREE_WHEEL = true;

**How it works:**// In build.gradle

// Which motor ports are your encoders plugged into?

public static String LEFT_ENCODER_PORT   = "frontLeft";1. Monitors acceleration (sudden change = push detected)repositories {

public static String RIGHT_ENCODER_PORT  = "frontRight";

public static String CENTER_ENCODER_PORT = "backLeft";2. Predicts position X milliseconds in the future    maven { url 'https://jitpack.io' }

```

3. Applies counter-force immediately}

### Step 4: Deploy & Drive

4. Result: **50ms+ faster response** without sacrificing accuracy

1. Deploy to robot

2. Select **"Novad TeleOp"** on Driver Stationdependencies {

3. Drive! 🎮

## 🎯 Features    implementation 'com.github.novad:novad:1.0.0'

## Controls

}

| Button | Action |

|--------|--------|- 🚀 **Predictive Defense** - Counter pushes before they happen```

| Left Stick | Drive/Strafe |

| Right Stick | Rotate |- 🛡️ **Push Resistance** - Automatically resist when opponents push your robot

| A | Toggle position lock (max defense) |

| B | Toggle defense on/off |- 🔒 **Position Lock** - Lock your robot in place for maximum resistance### 2. Configure Your Robot



## Tuning- 🎮 **Driver Override** - Instantly disables when driver touches joysticks



Open FTC Dashboard while running:- ⚡ **Instant Boost** - Multiplies response when impact detectedEdit `NovadConstants.java`:

- **On robot WiFi:** `http://192.168.43.1:8080/dash`

- **Or use:** [panels.bylazar.com](https://panels.bylazar.com)- 📊 **FTC Dashboard** - Live tune ALL values in real-time



Find `NovadSetup` in the sidebar and adjust:- 📍 **Pinpoint Support** - Works with GoBilda Pinpoint odometry```java

- `POSITION_P` - How hard it resists being pushed (default: 0.046)

- `HEADING_P` - How hard it resists rotation (default: 0.67)public static final DriveType DRIVE_TYPE = DriveType.MECANUM_4_MOTOR;



## Troubleshooting## 📦 Installationpublic static final OdometryType ODOMETRY_TYPE = OdometryType.THREE_WHEEL;



**Robot drives backwards?**```

→ Flip the `_REVERSED` values in NovadSetup.java

1. **Clone or download this repo**

**Robot doesn't resist pushes?**

→ Increase `POSITION_P` in Dashboard2. **Copy the `novad` folder** to your TeamCode:### 3. Use in TeleOp



**Robot oscillates/shakes?**   ```

→ Decrease `POSITION_P` or increase `POSITION_D`

   TeamCode/src/main/java/org/firstinspires/ftc/teamcode/novad/```java

---

   ```// Initialize

MIT License • Built for FTC teams 🤖

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
