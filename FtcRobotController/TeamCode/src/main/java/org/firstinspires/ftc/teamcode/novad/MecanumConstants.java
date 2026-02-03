package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Mecanum drivetrain configuration constants
 */
public class MecanumConstants {
    private double maxPower = 1.0;
    private String leftFrontMotorName = "frontLeft";
    private String leftRearMotorName = "backLeft";
    private String rightFrontMotorName = "frontRight";
    private String rightRearMotorName = "backRight";
    private DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    private DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    private DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    private DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    private boolean useBrakeMode = true;

    public MecanumConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public MecanumConstants leftFrontMotorName(String name) {
        this.leftFrontMotorName = name;
        return this;
    }

    public MecanumConstants leftRearMotorName(String name) {
        this.leftRearMotorName = name;
        return this;
    }

    public MecanumConstants rightFrontMotorName(String name) {
        this.rightFrontMotorName = name;
        return this;
    }

    public MecanumConstants rightRearMotorName(String name) {
        this.rightRearMotorName = name;
        return this;
    }

    public MecanumConstants leftFrontMotorDirection(DcMotorSimple.Direction direction) {
        this.leftFrontMotorDirection = direction;
        return this;
    }

    public MecanumConstants leftRearMotorDirection(DcMotorSimple.Direction direction) {
        this.leftRearMotorDirection = direction;
        return this;
    }

    public MecanumConstants rightFrontMotorDirection(DcMotorSimple.Direction direction) {
        this.rightFrontMotorDirection = direction;
        return this;
    }

    public MecanumConstants rightRearMotorDirection(DcMotorSimple.Direction direction) {
        this.rightRearMotorDirection = direction;
        return this;
    }

    public MecanumConstants useBrakeMode(boolean useBrakeMode) {
        this.useBrakeMode = useBrakeMode;
        return this;
    }

    // Getters
    public double getMaxPower() { return maxPower; }
    public String getLeftFrontMotorName() { return leftFrontMotorName; }
    public String getLeftRearMotorName() { return leftRearMotorName; }
    public String getRightFrontMotorName() { return rightFrontMotorName; }
    public String getRightRearMotorName() { return rightRearMotorName; }
    public DcMotorSimple.Direction getLeftFrontMotorDirection() { return leftFrontMotorDirection; }
    public DcMotorSimple.Direction getLeftRearMotorDirection() { return leftRearMotorDirection; }
    public DcMotorSimple.Direction getRightFrontMotorDirection() { return rightFrontMotorDirection; }
    public DcMotorSimple.Direction getRightRearMotorDirection() { return rightRearMotorDirection; }
    public boolean getUseBrakeMode() { return useBrakeMode; }
}
