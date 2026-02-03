package org.firstinspires.ftc.teamcode.novad;

import com.acmerobotics.dashboard.config.Config;

/**
 * PIDF Coefficients for controllers
 */
@Config
public class PIDFCoefficients {
    public double p;
    public double i;
    public double d;
    public double f;

    public PIDFCoefficients(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    public PIDFCoefficients() {
        this(0, 0, 0, 0);
    }
}
