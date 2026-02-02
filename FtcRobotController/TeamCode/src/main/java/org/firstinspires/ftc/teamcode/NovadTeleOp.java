package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.novad.Novad;

/**
 * EXAMPLE: How to use Novad in your TeleOp
 * 
 * Copy the pattern below into YOUR TeleOp.
 */
@TeleOp(name = "Novad Example", group = "Examples")
public class NovadTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        
        // ══════════════════════════════════════════════════════════════
        // INIT: Create Novad (reads config from NovadSetup.java)
        // ══════════════════════════════════════════════════════════════
        Novad novad = Novad.create(hardwareMap);
        
        telemetry.addLine("Novad Ready");
        telemetry.update();
        waitForStart();

        // ══════════════════════════════════════════════════════════════
        // LOOP: Use novad.defense() instead of regular drive code
        // ══════════════════════════════════════════════════════════════
        while (opModeIsActive()) {
            
            // DEFENSE MODE: Drive with push resistance
            // This replaces your normal drivetrain.drive() call
            novad.defense(
                gamepad1.left_stick_x,     // strafe
                -gamepad1.left_stick_y,    // forward (negate because gamepad Y is inverted)
                gamepad1.right_stick_x     // rotate
            );
            
            // LOCKDOWN: Bind to any button you want
            // Robot becomes immovable when held
            if (gamepad1.a) {
                novad.lockdown();
            }
            if (gamepad1.b) {
                novad.unlock();
            }
            
            // Optional: reset heading for field-centric
            if (gamepad1.y) {
                novad.resetHeading();
            }
            
            // Telemetry
            telemetry.addData("Mode", novad.isLocked() ? "LOCKDOWN" : "DEFENSE");
            telemetry.addData("Pos", novad.getPose());
            telemetry.update();
        }
    }
}
