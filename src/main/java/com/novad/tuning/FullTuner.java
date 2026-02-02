package com.novad.tuning;

import com.novad.Novad;
import com.novad.interfaces.NovadDrivetrain;
import com.novad.interfaces.NovadOdometry;

/**
 * FullTuner - exposes position, velocity and heading gains for complete tuning.
 * Use this in conjunction with Panels/FTC Dashboard. Public static fields are easy
 * to bind to dashboard sliders.
 */
public class FullTuner {

    // Position
    public static double POS_KP = 0.05;
    public static double POS_KI = 0.000;
    public static double POS_KD = 0.005;

    // Velocity
    public static double VEL_KP = 0.02;
    public static double VEL_KI = 0.0;
    public static double VEL_KD = 0.001;

    // Heading
    public static double HEADING_KP = 0.01;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.001;

    private final Novad novad;

    public FullTuner(NovadOdometry odometry, NovadDrivetrain drivetrain) {
        this.novad = new Novad(odometry, drivetrain);
        applyGains();
    }

    public void applyGains() {
        novad.setPositionPID(POS_KP, POS_KI, POS_KD);
        novad.setVelocityPID(VEL_KP, VEL_KI, VEL_KD);
        novad.setHeadingGains(HEADING_KP, HEADING_KI, HEADING_KD);
    }

    public void loop() {
        applyGains();
        novad.defense(0.0, 0.0, 0.0);
    }
}
