package org.firstinspires.ftc.teamcode.novad;

/**
 * Defense mode settings
 */
public class DefenseSettings {
    private double maxCorrectionPower = 0.8;
    private double joystickDeadzone = 0.05;
    private boolean useFieldCentric = false;

    public DefenseSettings maxCorrectionPower(double power) {
        this.maxCorrectionPower = power;
        return this;
    }

    public DefenseSettings joystickDeadzone(double deadzone) {
        this.joystickDeadzone = deadzone;
        return this;
    }

    public DefenseSettings useFieldCentric(boolean useFieldCentric) {
        this.useFieldCentric = useFieldCentric;
        return this;
    }

    // Getters
    public double getMaxCorrectionPower() { return maxCorrectionPower; }
    public double getJoystickDeadzone() { return joystickDeadzone; }
    public boolean getUseFieldCentric() { return useFieldCentric; }
}
