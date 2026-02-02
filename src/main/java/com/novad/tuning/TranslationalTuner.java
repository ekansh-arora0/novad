package com.novad.tuning;

import com.novad.Novad;
import com.novad.interfaces.NovadDrivetrain;
import com.novad.interfaces.NovadOdometry;
import com.novad.util.Vector2D;

/**
 * TranslationalTuner - exposes translational (X/Y) PID gains for live tuning
 * This acts like an OpMode wrapper: update public fields from Panels and call loop().
 */
public class TranslationalTuner {

    public static double POS_KP = 0.05;
    public static double POS_KI = 0.0;
    public static double POS_KD = 0.002;

    private final Novad novad;
    private final NovadOdometry odometry;

    public TranslationalTuner(NovadOdometry odometry, NovadDrivetrain drivetrain) {
        this.odometry = odometry;
        this.novad = new Novad(odometry, drivetrain);
        novad.setPositionPID(POS_KP, POS_KI, POS_KD);
    }

    public void loop() {
        // Sync gains from dashboard fields
        novad.setPositionPID(POS_KP, POS_KI, POS_KD);

        // Call defense to apply corrections with zero driver input
        novad.defense(0.0, 0.0, 0.0);

        Vector2D pos = odometry.getPosition();
        System.out.printf("pos=%.2f,%.2f\n", pos.getX(), pos.getY());
    }
}
