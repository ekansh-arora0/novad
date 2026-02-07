package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.novad.Constants;
import org.firstinspires.ftc.teamcode.novad.Novad;

/**
 * EXAMPLE: How to use Novad in your TeleOp
 * 
 * Shows ALL usage patterns including button-triggered movements.
 */
@TeleOp(name = "Novad Example", group = "Examples")
public class NovadTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        
        // ══════════════════════════════════════════════════════════════
        // INIT: Create Novad from Constants.java
        // ══════════════════════════════════════════════════════════════
        Novad novad = Constants.createNovad(hardwareMap);
        
        telemetry.addLine("Novad v" + Novad.VERSION + " Ready");
        telemetry.addLine("A = Lockdown | B = Unlock");
        telemetry.update();
        waitForStart();

        // ══════════════════════════════════════════════════════════════
        // LOOP
        // ══════════════════════════════════════════════════════════════
        while (opModeIsActive()) {
            
            // ──────────────────────────────────────────────────────────
            // OPTION 1: Simple - Pass gamepad (recommended)
            // Automatically reads ALL inputs including buttons
            // ──────────────────────────────────────────────────────────
            // novad.defense(gamepad1);
            
            // ──────────────────────────────────────────────────────────
            // OPTION 2: Joystick values only (legacy)
            // WARNING: Won't catch button-triggered movements!
            // ──────────────────────────────────────────────────────────
            // novad.defense(
            //     gamepad1.left_stick_x,
            //     -gamepad1.left_stick_y,
            //     gamepad1.right_stick_x
            // );
            
            // ──────────────────────────────────────────────────────────
            // OPTION 3: Motor powers (best for custom controls)
            // Use this if you have button-triggered movements
            // ──────────────────────────────────────────────────────────
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;
            
            // Example: Button-triggered auto-align or macro
            if (gamepad1.dpad_up) {
                forward = 0.5;  // Move forward at 50% power
                strafe = 0;
                rotate = 0;
            }
            if (gamepad1.dpad_down) {
                forward = -0.5;  // Move backward
                strafe = 0;
                rotate = 0;
            }
            if (gamepad1.dpad_left) {
                strafe = -0.5;  // Strafe left
                forward = 0;
                rotate = 0;
            }
            if (gamepad1.dpad_right) {
                strafe = 0.5;  // Strafe right
                forward = 0;
                rotate = 0;
            }
            
            // Calculate motor powers
            double lf = forward + strafe + rotate;
            double rf = forward - strafe - rotate;
            double lb = forward - strafe + rotate;
            double rb = forward + strafe - rotate;
            
            // Normalize
            double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), 
                         Math.max(Math.abs(lb), Math.abs(rb))));
            if (max > 1.0) {
                lf /= max;
                rf /= max;
                lb /= max;
                rb /= max;
            }
            
            // Pass motor powers - Novad will add corrections
            novad.defenseWithMotors(lf, rf, lb, rb);
            
            // ──────────────────────────────────────────────────────────
            // LOCKDOWN MODE
            // ──────────────────────────────────────────────────────────
            if (gamepad1.a) {
                novad.lockdown();
            }
            if (gamepad1.b) {
                novad.unlock();
            }
            
            // ──────────────────────────────────────────────────────────
            // TELEMETRY
            // ──────────────────────────────────────────────────────────
            telemetry.addData("Mode", novad.isLocked() ? "🔒 LOCKDOWN" : "🛡️ DEFENSE");
            telemetry.addData("Position", novad.getPose());
            telemetry.addData("Loop Time", "%.1f ms", novad.getLoopTimeMs());
            telemetry.update();
        }
    }
}
