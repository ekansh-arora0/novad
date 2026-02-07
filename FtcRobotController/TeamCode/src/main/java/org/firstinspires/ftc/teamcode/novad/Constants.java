package org.firstinspires.ftc.teamcode.novad;

// FTC Dashboard (uncomment if maven.brott.dev is reachable)
// import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Novad Constants - Configure your robot here
 * 
 * TUNING: Edit the PIDF values below and re-deploy to test.
 * If FTC Dashboard works, uncomment @Config for live tuning.
 */
// @Config  // Uncomment if FTC Dashboard is available
public class Constants {

    // ═══════════════════════════════════════════════════════════════════════════
    //                           PIDF COEFFICIENTS
    // ═══════════════════════════════════════════════════════════════════════════

    public static PIDFCoefficients translationalPIDF = new PIDFCoefficients(0.065, 0, 0.033, 0.001);
    public static PIDFCoefficients headingPIDF = new PIDFCoefficients(1, 0, 0, 0.01);

    // ═══════════════════════════════════════════════════════════════════════════
    //                         DRIVETRAIN CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════════

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("front_left_drive")
            .leftRearMotorName("back_left_drive")
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeMode(true);

    // ═══════════════════════════════════════════════════════════════════════════
    //                        LOCALIZER CONSTANTS
    // ═══════════════════════════════════════════════════════════════════════════

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // ═══════════════════════════════════════════════════════════════════════════
    //                         DEFENSE SETTINGS
    // ═══════════════════════════════════════════════════════════════════════════

    public static DefenseSettings defenseSettings = new DefenseSettings()
            .maxCorrectionPower(0.8)
            .joystickDeadzone(0.05)
            .useFieldCentric(false);

    // ═══════════════════════════════════════════════════════════════════════════
    //                         CREATE NOVAD INSTANCE
    // ═══════════════════════════════════════════════════════════════════════════

    public static Novad createNovad(HardwareMap hardwareMap) {
        return new NovadBuilder(hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .translationalPIDF(translationalPIDF)
                .headingPIDF(headingPIDF)
                .defenseSettings(defenseSettings)
                .build();
    }
}
