package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.novad.Novad;
import org.firstinspires.ftc.teamcode.novad.adapters.MecanumDriveAdapter;
import org.firstinspires.ftc.teamcode.novad.adapters.PinpointOdometry;
import org.firstinspires.ftc.teamcode.novad.adapters.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════════╗
 * ║                         NOVAD TELEOP                                          ║
 * ║                                                                               ║
 * ║  This is ready to use! Just configure NovadSetup.java first.                  ║
 * ║                                                                               ║
 * ║  CONTROLS:                                                                    ║
 * ║  • Left Stick  = Drive/Strafe                                                 ║
 * ║  • Right Stick = Rotate                                                       ║
 * ║  • A Button    = Toggle position lock (max defense)                           ║
 * ║  • B Button    = Toggle defense on/off                                        ║
 * ╚═══════════════════════════════════════════════════════════════════════════════╝
 */
@TeleOp(name = "Novad TeleOp", group = "Novad")
public class NovadTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ═══════════════════════════════════════════════════════════════════════
        // MOTORS - Uses names from NovadSetup
        // ═══════════════════════════════════════════════════════════════════════
        
        DcMotor frontLeft  = hardwareMap.get(DcMotor.class, NovadSetup.FRONT_LEFT);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, NovadSetup.FRONT_RIGHT);
        DcMotor backLeft   = hardwareMap.get(DcMotor.class, NovadSetup.BACK_LEFT);
        DcMotor backRight  = hardwareMap.get(DcMotor.class, NovadSetup.BACK_RIGHT);

        // Set directions from NovadSetup
        frontLeft.setDirection(NovadSetup.FRONT_LEFT_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(NovadSetup.FRONT_RIGHT_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(NovadSetup.BACK_LEFT_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(NovadSetup.BACK_RIGHT_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // ═══════════════════════════════════════════════════════════════════════
        // ODOMETRY - Auto-selects based on NovadSetup
        // ═══════════════════════════════════════════════════════════════════════
        
        NovadOdometry odometry;
        String odomType;
        
        if (NovadSetup.USE_PINPOINT) {
            odometry = new PinpointOdometry(hardwareMap, NovadSetup.PINPOINT_NAME);
            odomType = "Pinpoint";
        } else {
            odometry = new ThreeWheelOdometry(
                hardwareMap,
                NovadSetup.LEFT_ENCODER_PORT,
                NovadSetup.RIGHT_ENCODER_PORT,
                NovadSetup.CENTER_ENCODER_PORT,
                NovadSetup.LEFT_ENCODER_REVERSED,
                NovadSetup.RIGHT_ENCODER_REVERSED,
                NovadSetup.CENTER_ENCODER_REVERSED,
                NovadSetup.WHEEL_DIAMETER_INCHES,
                8192, // Encoder ticks
                NovadSetup.TRACK_WIDTH_INCHES,
                NovadSetup.FORWARD_OFFSET_INCHES
            );
            odomType = "Three-Wheel";
        }

        // ═══════════════════════════════════════════════════════════════════════
        // NOVAD
        // ═══════════════════════════════════════════════════════════════════════
        
        MecanumDriveAdapter drivetrain = new MecanumDriveAdapter(
            frontLeft, frontRight, backLeft, backRight
        );
        
        Novad novad = new Novad(odometry, drivetrain);

        // ═══════════════════════════════════════════════════════════════════════
        // READY
        // ═══════════════════════════════════════════════════════════════════════
        
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("    🛡️ NOVAD READY");
        telemetry.addLine("════════════════════════════════");
        telemetry.addLine("");
        telemetry.addData("Odometry", odomType);
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addLine("  A = Toggle Position Lock");
        telemetry.addLine("  B = Toggle Defense");
        telemetry.addLine("");
        telemetry.addLine("Tune values in FTC Dashboard!");
        telemetry.update();

        waitForStart();

        boolean lastA = false, lastB = false;
        boolean defenseOn = true;

        while (opModeIsActive()) {
            // ═══════════════════════════════════════════════════════════════════
            // UPDATE PID FROM DASHBOARD (live tuning!)
            // ═══════════════════════════════════════════════════════════════════
            
            novad.setPositionPID(NovadSetup.POSITION_P, NovadSetup.POSITION_I, NovadSetup.POSITION_D);
            novad.setHeadingPID(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D);
            novad.setPredictiveEnabled(NovadSetup.PREDICTIVE_ENABLED);
            novad.setPredictionLookahead(NovadSetup.PREDICTION_MS);
            novad.setInstantBoostMultiplier(NovadSetup.BOOST_MULTIPLIER);

            // ═══════════════════════════════════════════════════════════════════
            // BUTTONS
            // ═══════════════════════════════════════════════════════════════════
            
            // A = Toggle position lock
            if (gamepad1.a && !lastA) {
                novad.togglePositionLock();
            }
            lastA = gamepad1.a;
            
            // B = Toggle defense
            if (gamepad1.b && !lastB) {
                defenseOn = !defenseOn;
                if (!defenseOn) novad.disable();
            }
            lastB = gamepad1.b;

            // ═══════════════════════════════════════════════════════════════════
            // DRIVE
            // ═══════════════════════════════════════════════════════════════════
            
            if (defenseOn) {
                // Novad handles everything!
                novad.defense(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x
                );
            } else {
                // Manual drive
                drivetrain.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
                );
            }

            // ═══════════════════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════════════════
            
            telemetry.addLine("════════ NOVAD ════════");
            telemetry.addData("Defense", defenseOn ? "🟢 ON" : "🔴 OFF");
            telemetry.addData("Position Lock", novad.isPositionLocked() ? "🔒 LOCKED" : "🔓 FREE");
            telemetry.addData("Predictive", novad.isPredictiveActive() ? "⚡ ACTIVE" : "—");
            telemetry.addLine("");
            telemetry.addData("X", "%.1f in", odometry.getPosition().x);
            telemetry.addData("Y", "%.1f in", odometry.getPosition().y);
            telemetry.addData("Heading", "%.0f°", Math.toDegrees(odometry.getHeading()));
            telemetry.update();
        }
    }
}
