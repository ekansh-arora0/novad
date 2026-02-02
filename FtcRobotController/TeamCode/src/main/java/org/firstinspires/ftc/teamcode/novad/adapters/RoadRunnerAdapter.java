package org.firstinspires.ftc.teamcode.novad.adapters;

import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;
import org.firstinspires.ftc.teamcode.novad.util.Vector2D;

/**
 * Adapter for RoadRunner Localizer
 * 
 * Use this class to connect Novad to your RoadRunner localizer.
 * 
 * Example:
 * <pre>
 * SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
 * NovadOdometry odometry = new RoadRunnerAdapter(drive);
 * Novad novad = new Novad(odometry, drivetrain);
 * </pre>
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class RoadRunnerAdapter implements NovadOdometry {

    // RoadRunner drive instance
    private final Object drive;

    // Cached values
    private double x, y, heading;
    private double vx, vy, omega;

    // Previous values for velocity calculation
    private double prevX, prevY, prevHeading;
    private long prevTime;

    /**
     * Create an adapter for RoadRunner
     * @param drive Your RoadRunner MecanumDrive or SampleMecanumDrive instance
     */
    public RoadRunnerAdapter(Object drive) {
        this.drive = drive;
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;
        this.prevX = 0;
        this.prevY = 0;
        this.prevHeading = 0;
        this.prevTime = System.nanoTime();
    }

    @Override
    public void update() {
        try {
            // Call drive's update method
            java.lang.reflect.Method updateMethod = drive.getClass().getMethod("update");
            updateMethod.invoke(drive);

            // Get pose estimate
            Object poseEstimate = null;
            try {
                java.lang.reflect.Method getPoseMethod = drive.getClass().getMethod("getPoseEstimate");
                poseEstimate = getPoseMethod.invoke(drive);
            } catch (NoSuchMethodException e) {
                // Try alternative
                try {
                    java.lang.reflect.Method getLocalizerMethod = drive.getClass().getMethod("getLocalizer");
                    Object localizer = getLocalizerMethod.invoke(drive);
                    if (localizer != null) {
                        java.lang.reflect.Method getPoseMethod = localizer.getClass().getMethod("getPoseEstimate");
                        poseEstimate = getPoseMethod.invoke(localizer);
                    }
                } catch (Exception e2) {
                    // Fallback
                }
            }

            if (poseEstimate != null) {
                // Save previous values
                prevX = x;
                prevY = y;
                prevHeading = heading;
                long currentTime = System.nanoTime();

                // Extract pose values
                Class<?> poseClass = poseEstimate.getClass();

                // RoadRunner Pose2d typically has getX(), getY(), getHeading()
                try {
                    x = (Double) poseClass.getMethod("getX").invoke(poseEstimate);
                    y = (Double) poseClass.getMethod("getY").invoke(poseEstimate);
                    heading = (Double) poseClass.getMethod("getHeading").invoke(poseEstimate);
                } catch (NoSuchMethodException e) {
                    // Try accessing fields directly
                    try {
                        x = poseClass.getField("x").getDouble(poseEstimate);
                        y = poseClass.getField("y").getDouble(poseEstimate);
                        heading = poseClass.getField("heading").getDouble(poseEstimate);
                    } catch (NoSuchFieldException e2) {
                        // Try component methods (Kotlin data class)
                        try {
                            x = (Double) poseClass.getMethod("component1").invoke(poseEstimate);
                            y = (Double) poseClass.getMethod("component2").invoke(poseEstimate);
                            heading = (Double) poseClass.getMethod("component3").invoke(poseEstimate);
                        } catch (Exception e3) {
                            // Unable to extract
                        }
                    }
                }

                // Calculate velocity
                double dt = (currentTime - prevTime) / 1_000_000_000.0;
                if (dt > 0.001) {
                    vx = (x - prevX) / dt;
                    vy = (y - prevY) / dt;
                    omega = normalizeAngle(heading - prevHeading) / dt;
                }
                prevTime = currentTime;
            }

            // Try to get velocity directly from RoadRunner
            try {
                java.lang.reflect.Method getVelMethod = drive.getClass().getMethod("getPoseVelocity");
                Object velocity = getVelMethod.invoke(drive);
                if (velocity != null) {
                    Class<?> velClass = velocity.getClass();
                    vx = (Double) velClass.getMethod("getX").invoke(velocity);
                    vy = (Double) velClass.getMethod("getY").invoke(velocity);
                    omega = (Double) velClass.getMethod("getHeading").invoke(velocity);
                }
            } catch (Exception e) {
                // Use calculated velocity
            }

        } catch (Exception e) {
            // Reflection failed
        }
    }

    @Override
    public Vector2D getPosition() {
        return new Vector2D(x, y);
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getHeading() {
        return heading;
    }

    @Override
    public Vector2D getVelocity() {
        return new Vector2D(vx, vy);
    }

    @Override
    public double getXVelocity() {
        return vx;
    }

    @Override
    public double getYVelocity() {
        return vy;
    }

    @Override
    public double getAngularVelocity() {
        return omega;
    }

    @Override
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.prevX = x;
        this.prevY = y;
        this.prevHeading = heading;

        // Try to set pose on RoadRunner drive
        try {
            // Find Pose2d class
            Class<?> pose2dClass = Class.forName("com.acmerobotics.roadrunner.geometry.Pose2d");
            Object pose = pose2dClass.getConstructor(double.class, double.class, double.class)
                .newInstance(x, y, heading);

            java.lang.reflect.Method setPoseMethod = drive.getClass().getMethod("setPoseEstimate", pose2dClass);
            setPoseMethod.invoke(drive, pose);
        } catch (Exception e) {
            // setPose not available
        }
    }

    /**
     * Normalize angle to [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Get the underlying RoadRunner drive
     * @return Drive instance
     */
    public Object getDrive() {
        return drive;
    }
}
