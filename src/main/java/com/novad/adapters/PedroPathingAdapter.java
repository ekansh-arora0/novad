package com.novad.adapters;

import com.novad.interfaces.NovadOdometry;
import com.novad.util.Vector2D;

/**
 * Adapter for Pedro Pathing Follower
 * 
 * Use this class to connect Novad to your Pedro Pathing localizer.
 * 
 * Example:
 * <pre>
 * Follower follower = new Follower(hardwareMap);
 * NovadOdometry odometry = new PedroPathingAdapter(follower);
 * Novad novad = new Novad(odometry, drivetrain);
 * </pre>
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class PedroPathingAdapter implements NovadOdometry {

    // Pedro Pathing Follower instance
    // Using Object type to avoid compile-time dependency
    private final Object follower;

    // Cached values
    private double x, y, heading;
    private double vx, vy, omega;

    // Reflection method references (resolved at runtime)
    private java.lang.reflect.Method getPoseMethod;
    private java.lang.reflect.Method getVelocityMethod;
    private java.lang.reflect.Method updateMethod;

    /**
     * Create an adapter for Pedro Pathing
     * @param follower Your Pedro Pathing Follower instance
     */
    public PedroPathingAdapter(Object follower) {
        this.follower = follower;
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;

        // Resolve methods at runtime to avoid compile-time dependency
        try {
            Class<?> followerClass = follower.getClass();
            
            // Try to find getPose method
            try {
                getPoseMethod = followerClass.getMethod("getPose");
            } catch (NoSuchMethodException e) {
                // Try alternative method names
                try {
                    getPoseMethod = followerClass.getMethod("getLocalizer");
                } catch (NoSuchMethodException e2) {
                    // Will use fallback
                }
            }

            // Try to find update method
            try {
                updateMethod = followerClass.getMethod("update");
            } catch (NoSuchMethodException e) {
                // No update method needed
            }

        } catch (Exception e) {
            // Continue with fallback behavior
        }
    }

    @Override
    public void update() {
        try {
            // Call follower's update if available
            if (updateMethod != null) {
                updateMethod.invoke(follower);
            }

            // Get pose from follower
            if (getPoseMethod != null) {
                Object pose = getPoseMethod.invoke(follower);
                if (pose != null) {
                    // Extract x, y, heading from pose
                    // Pedro Pathing uses Pose2d with x, y, heading fields
                    Class<?> poseClass = pose.getClass();
                    
                    try {
                        // Try getX(), getY(), getHeading() methods
                        x = (Double) poseClass.getMethod("getX").invoke(pose);
                        y = (Double) poseClass.getMethod("getY").invoke(pose);
                        heading = (Double) poseClass.getMethod("getHeading").invoke(pose);
                    } catch (NoSuchMethodException e) {
                        // Try x, y, heading fields
                        try {
                            x = poseClass.getField("x").getDouble(pose);
                            y = poseClass.getField("y").getDouble(pose);
                            heading = poseClass.getField("heading").getDouble(pose);
                        } catch (NoSuchFieldException e2) {
                            // Unable to extract pose
                        }
                    }
                }
            }

            // Try to get velocity
            try {
                java.lang.reflect.Method getVelMethod = follower.getClass().getMethod("getVelocity");
                Object velocity = getVelMethod.invoke(follower);
                if (velocity != null) {
                    Class<?> velClass = velocity.getClass();
                    try {
                        vx = (Double) velClass.getMethod("getX").invoke(velocity);
                        vy = (Double) velClass.getMethod("getY").invoke(velocity);
                    } catch (Exception e) {
                        // Velocity not available
                    }
                }
            } catch (Exception e) {
                // Velocity method not available
            }

        } catch (Exception e) {
            // Reflection failed - values unchanged
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

        // Try to set pose on follower
        try {
            java.lang.reflect.Method setPoseMethod = follower.getClass().getMethod("setPose", double.class, double.class, double.class);
            setPoseMethod.invoke(follower, x, y, heading);
        } catch (Exception e) {
            // setPose not available or different signature
        }
    }

    /**
     * Get the underlying Pedro Pathing Follower
     * @return Follower instance
     */
    public Object getFollower() {
        return follower;
    }
}
