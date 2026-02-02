package com.novad.config;

/**
 * PIDF Coefficients for PID control with feedforward
 * 
 * P = Proportional (reacts to current error)
 * I = Integral (reacts to accumulated error)
 * D = Derivative (reacts to rate of change)
 * F = Feedforward (constant boost)
 */
public class PIDFCoefficients {
    
    public final double p;
    public final double i;
    public final double d;
    public final double f;
    
    public PIDFCoefficients(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
    
    public PIDFCoefficients(double p, double i, double d) {
        this(p, i, d, 0.0);
    }
    
    @Override
    public String toString() {
        return String.format("PIDF(%.4f, %.4f, %.4f, %.4f)", p, i, d, f);
    }
    
    /**
     * Create a copy with modified P value
     */
    public PIDFCoefficients withP(double newP) {
        return new PIDFCoefficients(newP, this.i, this.d, this.f);
    }
    
    /**
     * Create a copy with modified I value
     */
    public PIDFCoefficients withI(double newI) {
        return new PIDFCoefficients(this.p, newI, this.d, this.f);
    }
    
    /**
     * Create a copy with modified D value
     */
    public PIDFCoefficients withD(double newD) {
        return new PIDFCoefficients(this.p, this.i, newD, this.f);
    }
    
    /**
     * Create a copy with modified F value
     */
    public PIDFCoefficients withF(double newF) {
        return new PIDFCoefficients(this.p, this.i, this.d, newF);
    }
}
