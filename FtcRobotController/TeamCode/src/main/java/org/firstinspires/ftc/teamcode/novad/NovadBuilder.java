package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Builder for creating Novad instances
 */
public class NovadBuilder {
    private final HardwareMap hardwareMap;
    private MecanumConstants mecanumConstants;
    private PinpointConstants pinpointConstants;
    private PIDFCoefficients translationalPIDF;
    private PIDFCoefficients headingPIDF;
    private DefenseSettings defenseSettings;

    public NovadBuilder(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public NovadBuilder mecanumDrivetrain(MecanumConstants constants) {
        this.mecanumConstants = constants;
        return this;
    }

    public NovadBuilder pinpointLocalizer(PinpointConstants constants) {
        this.pinpointConstants = constants;
        return this;
    }

    public NovadBuilder translationalPIDF(PIDFCoefficients coefficients) {
        this.translationalPIDF = coefficients;
        return this;
    }

    public NovadBuilder headingPIDF(PIDFCoefficients coefficients) {
        this.headingPIDF = coefficients;
        return this;
    }

    public NovadBuilder defenseSettings(DefenseSettings settings) {
        this.defenseSettings = settings;
        return this;
    }

    public Novad build() {
        if (mecanumConstants == null) {
            throw new IllegalStateException("MecanumConstants must be set");
        }
        if (pinpointConstants == null) {
            throw new IllegalStateException("PinpointConstants must be set");
        }
        if (translationalPIDF == null) {
            translationalPIDF = new PIDFCoefficients(0.1, 0, 0.01, 0);
        }
        if (headingPIDF == null) {
            headingPIDF = new PIDFCoefficients(1, 0, 0.01, 0);
        }
        if (defenseSettings == null) {
            defenseSettings = new DefenseSettings();
        }

        return new Novad(
            hardwareMap,
            mecanumConstants,
            pinpointConstants,
            translationalPIDF,
            headingPIDF,
            defenseSettings
        );
    }
}
