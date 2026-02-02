# 🛡️ Novad - FTC Defense Library## TeamCode Module



**Stop getting pushed around.** Novad is a defense library for FTC robots that automatically resists being pushed by other robots during matches.Welcome!



## How It WorksThis module, TeamCode, is the place where you will write/paste the code for your team's

robot controller App. This module is currently empty (a clean slate) but the

1. **Odometry Detection**: Novad monitors your robot's position using odometryprocess for adding OpModes is straightforward.

2. **Push Detection**: When your robot moves without joystick input, Novad knows you're being pushed

3. **Counter-Force**: Novad automatically applies motor power to resist the push## Creating your own OpModes

4. **Driver Override**: As soon as you touch the joysticks, Novad steps aside

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

## Quick Start

Sample opmodes exist in the FtcRobotController module.

### 1. Configure Your RobotTo locate these samples, find the FtcRobotController module in the "Project/Android" tab.



```javaExpand the following tree elements:

NovadConfig config = new NovadConfig.Builder() FtcRobotController/java/org.firstinspires.ftc.robotcontroller/external/samples

    // Motor names (from your robot configuration)

    .leftFrontMotorName("left_front_drive")### Naming of Samples

    .leftRearMotorName("left_back_drive")

    .rightFrontMotorName("right_front_drive")To gain a better understanding of how the samples are organized, and how to interpret the

    .rightRearMotorName("right_back_drive")naming system, it will help to understand the conventions that were used during their creation.

    

    // Motor directions (left side typically reversed)These conventions are described (in detail) in the sample_conventions.md file in this folder.

    .leftFrontMotorDirection(MotorDirection.REVERSE)

    .leftRearMotorDirection(MotorDirection.REVERSE)To summarize: A range of different samples classes will reside in the java/external/samples.

    .rightFrontMotorDirection(MotorDirection.FORWARD)The class names will follow a naming convention which indicates the purpose of each class.

    .rightRearMotorDirection(MotorDirection.FORWARD)The prefix of the name will be one of the following:

    

    // PIDF tuning (use the tuner OpModes to find these!)Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure

    .translationalPIDFCoefficients(new PIDFCoefficients(0.046, 0, 0.01, 0.02))            of a particular style of OpMode.  These are bare bones examples.

    .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0, 0.006, 0.02))

    Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.

    .build();            It is not intended to drive a functioning robot, it is simply showing the minimal code

```            required to read and display the sensor values.



### 2. Create NovadRobot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.

            It may be used to provide a common baseline driving OpMode, or

```java            to demonstrate how a particular sensor or concept can be used to navigate.

// Set up odometry

ThreeWheelOdometry odometry = new ThreeWheelOdometry(Concept:	This is a sample OpMode that illustrates performing a specific function or concept.

    () -> leftEncoder.getCurrentPosition(),            These may be complex, but their operation should be explained clearly in the comments,

    () -> rightEncoder.getCurrentPosition(),            or the comments should reference an external doc, guide or tutorial.

    () -> centerEncoder.getCurrentPosition(),            Each OpMode should try to only demonstrate a single concept so they are easy to

    1.89,    // wheel diameter (inches)            locate based on their name.  These OpModes may not produce a drivable robot.

    8192,    // ticks per revolution

    14.0,    // track width (inches)After the prefix, other conventions will apply:

    6.0      // forward offset (inches)

);* Sensor class names are constructed as:    Sensor - Company - Type

* Robot class names are constructed as:     Robot - Mode - Action - OpModetype

// Set up drivetrain* Concept class names are constructed as:   Concept - Topic - OpModetype

MecanumDriveAdapter drivetrain = new MecanumDriveAdapter(

    frontLeft, frontRight, backLeft, backRightOnce you are familiar with the range of samples available, you can choose one to be the

);basis for your own robot.  In all cases, the desired sample(s) needs to be copied into

your TeamCode module to be used.

// Create Novad

Novad novad = new Novad(odometry, drivetrain, config);This is done inside Android Studio directly, using the following steps:

```

 1) Locate the desired sample class in the Project/Android tree.

### 3. Use in TeleOp

 2) Right click on the sample class and select "Copy"

```java

while (opModeIsActive()) { 3) Expand the  TeamCode/java folder

    // This one line handles everything!

    novad.defense( 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

        gamepad1.left_stick_x,    // Strafe

        -gamepad1.left_stick_y,   // Forward (inverted) 5) You will be prompted for a class name for the copy.

        gamepad1.right_stick_x    // Rotate    Choose something meaningful based on the purpose of this class.

    );    Start with a capital letter, and remember that there may be more similar classes later.

}

```Once your copy has been created, you should prepare it for use on your robot.

This is done by adjusting the OpMode's name, and enabling it to be displayed on the

## TuningDriver Station's OpMode list.



Novad includes tuning OpModes that work with FTC Dashboard:Each OpMode sample class begins with several lines of code like the ones shown below:



| OpMode | Purpose |```

|--------|---------| @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")

| `NovadHeadingTuner` | Tune rotation resistance | @Disabled

| `NovadTranslationalTuner` | Tune X/Y position hold |```

| `NovadFullTuner` | Fine-tune all parameters |

The name that will appear on the driver station's "opmode list" is defined by the code:

### Tuning Process ``name="Template: Linear OpMode"``

You can change what appears between the quotes to better describe your opmode.

1. **Deploy** the tuner OpMode to your robotThe "group=" portion of the code can be used to help organize your list of OpModes.

2. **Connect** to FTC Dashboard (`http://192.168.43.1:8080/dash` or [panels.bylazar.com](https://panels.bylazar.com))

3. **Run** the OpModeAs shown, the current OpMode will NOT appear on the driver station's OpMode list because of the

4. **Adjust** PID values in the dashboard  ``@Disabled`` annotation which has been included.

5. **Test** by pushing the robotThis line can simply be deleted , or commented out, to make the OpMode visible.

6. **Copy** final values to your TeleOp config



### Recommended Starting Values

## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

```java

// Translational (position hold)In some situations, you have multiple teams in your club and you want them to all share

POS_P = 0.046a common code organization, with each being able to *see* the others code but each having

POS_I = 0.0their own team module with their own code that they maintain themselves.

POS_D = 0.01

In this situation, you might wish to clone the TeamCode module, once for each of these teams.

// Heading (rotation hold)Each of the clones would then appear along side each other in the Android Studio module list,

HEADING_P = 0.67together with the FtcRobotController module (and the original TeamCode module).

HEADING_I = 0.0

HEADING_D = 0.006Selective Team phones can then be programmed by selecting the desired Module from the pulldown list

```prior to clicking to the green Run arrow.



## New Team FlowWarning:  This is not for the inexperienced Software developer.

You will need to be comfortable with File manipulations and managing Android Studio Modules.

If you're new to Novad, here's the complete process:These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.

 

```Also.. Make a full project backup before you start this :)

┌─────────────────────────────────────────────────────────────┐

│  1. INSTALL                                                 │To clone TeamCode, do the following:

│     Clone this repo or copy the novad folder to TeamCode    │

└─────────────────────────────────────────────────────────────┘Note: Some names start with "Team" and others start with "team".  This is intentional.

                              ↓

┌─────────────────────────────────────────────────────────────┐1)  Using your operating system file management tools, copy the whole "TeamCode"

│  2. CONFIGURE                                               │    folder to a sibling folder with a corresponding new name, eg: "Team0417".

│     Create NovadConfig with your motor names & directions   │

└─────────────────────────────────────────────────────────────┘2)  In the new Team0417 folder, delete the TeamCode.iml file.

                              ↓

┌─────────────────────────────────────────────────────────────┐3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder

│  3. SET UP ODOMETRY                                         │    to a matching name with a lowercase 'team' eg:  "team0417".

│     Use ThreeWheelOdometry or connect your existing system  │

└─────────────────────────────────────────────────────────────┘4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains

                              ↓         package="org.firstinspires.ftc.teamcode"

┌─────────────────────────────────────────────────────────────┐    to be

│  4. TUNE HEADING                                            │         package="org.firstinspires.ftc.team0417"

│     Run NovadHeadingTuner, adjust P/I/D until smooth        │

└─────────────────────────────────────────────────────────────┘5)  Add:    include ':Team0417' to the "/settings.gradle" file.

                              ↓    

┌─────────────────────────────────────────────────────────────┐6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""
│  5. TUNE TRANSLATIONAL                                      │
│     Run NovadTranslationalTuner, adjust P/I/D until smooth  │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  6. INTEGRATE                                               │
│     Add novad.defense() to your TeleOp loop                 │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│  7. TEST & COMPETE                                          │
│     Practice with defense, fine-tune as needed              │
└─────────────────────────────────────────────────────────────┘
```

## API Quick Reference

### Novad Class

| Method | Description |
|--------|-------------|
| `defense(x, y, rotation)` | Main method - handles defense + driver input |
| `lockPosition()` | Lock current position for max resistance |
| `unlockPosition()` | Release position lock |
| `togglePositionLock()` | Toggle lock on/off |
| `enable()` / `disable()` | Turn defense on/off |
| `setPositionPID(p, i, d)` | Set translational gains |
| `setHeadingPID(p, i, d)` | Set heading gains |
| `setVelocityPID(p, i, d)` | Set velocity gains |

### NovadConfig.Builder

| Method | Description |
|--------|-------------|
| `leftFrontMotorName(name)` | Motor name from config |
| `leftFrontMotorDirection(dir)` | FORWARD or REVERSE |
| `translationalPIDFCoefficients(pidf)` | Position hold tuning |
| `headingPIDFCoefficients(pidf)` | Rotation hold tuning |
| `movementThreshold(inches)` | Min movement to trigger |
| `maxCorrectionPower(0-1)` | Max correction power |

## File Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── novad/
│   ├── Novad.java              # Main entry point
│   ├── adapters/
│   │   ├── MecanumDriveAdapter.java
│   │   └── ThreeWheelOdometry.java
│   ├── config/
│   │   ├── NovadConfig.java    # Builder-style configuration
│   │   ├── PIDFCoefficients.java
│   │   └── MotorDirection.java
│   ├── controller/
│   │   └── DefenseController.java
│   ├── interfaces/
│   │   ├── NovadOdometry.java
│   │   └── NovadDrivetrain.java
│   ├── tuning/
│   │   ├── HeadingTuner.java
│   │   ├── TranslationalTuner.java
│   │   └── FullTuner.java
│   └── util/
│       ├── Vector2D.java
│       ├── PIDController.java
│       └── MathUtils.java
├── NovadHeadingTuner.java      # FTC Dashboard tuner
├── NovadTranslationalTuner.java
├── NovadFullTuner.java
└── NovadTeleOpExample.java     # Complete example
```

## Requirements

- FTC SDK 8.0+
- FTC Dashboard (for tuning)
- Three-wheel odometry (or compatible system)

## License

MIT License - Free for all FTC teams to use and modify.

---

**Built for FTC teams, by FTC teams. 🤖**
