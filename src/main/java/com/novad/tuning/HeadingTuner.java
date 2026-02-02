package com.novad.tuning;

import com.novad.Novad;
import com.novad.interfaces.NovadDrivetrain;
import com.novad.interfaces.NovadOdometry;
import com.novad.util.Vector2D;

/**
 * HeadingTuner - simple OpMode-style tuner that exposes heading PID to Panels
 * NOTE: This file is intended to be copied into an FTC TeamCode project and
 * used as an OpMode. It is written without direct SDK imports so it compiles
 * in this library workspace.
 */
public class HeadingTuner {

    // Public tuning fields that Panels/FTC Dashboard can edit (simulated here)
    public static double HEADING_KP = 0.01;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.001;

    // Target heading for tests (radians)
    public static double TARGET_HEADING = 0.0;

    private final Novad novad;
    private final NovadOdometry odometry;
    private final NovadDrivetrain drivetrain;

    public HeadingTuner(NovadOdometry odometry, NovadDrivetrain drivetrain) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        this.novad = new Novad(odometry, drivetrain);

        // Apply initial heading gains
        novad.setHeadingGains(HEADING_KP, HEADING_KI, HEADING_KD);
    }

    // Call this repeatedly in your loop. In a real OpMode, hook this into FTC Dashboard
    public void loop() {
        // Update gains from public fields so dashboard can change them live
        novad.setHeadingGains(HEADING_KP, HEADING_KI, HEADING_KD);

        // We use the Novad controller by passing zero driver translational input and
        // only a small rotation target — novad will resist deviation from TARGET_HEADING
        // The driver rotation input parameter is used to pass through user commands.
        novad.defense(0.0, 0.0, 0.0);

        // Telemetry placeholders (in real OpMode, publish to dashboard)
        Vector2D pos = odometry.getPosition();
        double heading = odometry.getHeading();

        // Example telemetry format (replace with actual dashboard calls):
        System.out.printf("pos=%.2f,%.2f heading=%.2f deg\n", pos.getX(), pos.getY(), Math.toDegrees(heading));
    }
}
