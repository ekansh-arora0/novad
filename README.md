# Novad - FTC Defense Library

**Stop Getting Pushed Around.**

Novad is a defense library for FTC robots that prevents opponents from pushing your robot around during matches. Using odometry to detect unwanted movement, Novad automatically applies counter-force to resist defense.

## Features

- 🛡️ **Push Resistance** - Automatically resist when opponents push your robot
- 🎯 **Position Lock** - Lock your robot in place for maximum resistance
- 🎮 **Driver Override** - Instantly disables when driver touches joysticks
- ⚡ **Smooth Ramp-Up** - Gradual power increase prevents jerky movements
- 🔧 **Easy Tuning** - Real-time PID tuning with panels.bylazar.com
- 🔌 **Universal** - Works with Pedro Pathing, RoadRunner, or any odometry

## Quick Start

### 1. Add Dependency

```gradle
// In build.gradle
repositories {
    maven { url 'https://jitpack.io' }
}

dependencies {
    implementation 'com.github.novad:novad:1.0.0'
}
```

### 2. Configure Your Robot

Edit `NovadConstants.java`:

```java
public static final DriveType DRIVE_TYPE = DriveType.MECANUM_4_MOTOR;
public static final OdometryType ODOMETRY_TYPE = OdometryType.THREE_WHEEL;
```

### 3. Use in TeleOp

```java
// Initialize
Novad novad = new Novad(odometry, drivetrain);

// In your loop - that's it!
novad.defense(
    gamepad1.left_stick_x,
    -gamepad1.left_stick_y,
    gamepad1.right_stick_x
);
```

## Documentation

Visit [novad.dev](https://novad.dev) for complete documentation, tuning guides, and examples.

## Tuning

1. Run the `DefenseTuning` OpMode
2. Open [panels.bylazar.com](https://panels.bylazar.com) (or `192.168.43.1:8080/dash`)
3. Have a partner push your robot
4. Adjust PID sliders until robot resists stably
5. Copy values to `NovadConstants.java`

## Integration Examples

### With Pedro Pathing

```java
Follower follower = new Follower(hardwareMap);
NovadOdometry odometry = new PedroPathingAdapter(follower);
Novad novad = new Novad(odometry, drivetrain);
```

### With RoadRunner

```java
SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
NovadOdometry odometry = new RoadRunnerAdapter(drive);
Novad novad = new Novad(odometry, drivetrain);
```

### Position Lock

```java
// Lock position when A pressed
if (gamepad1.a) {
    novad.lockPosition();
}

// Unlock when B pressed
if (gamepad1.b) {
    novad.unlockPosition();
}
```

## License

MIT License - see [LICENSE](LICENSE) for details.
