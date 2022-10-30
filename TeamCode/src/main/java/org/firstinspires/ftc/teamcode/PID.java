package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double errorSum;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    public double getOutput(double error, double dError) {
        errorSum += error;
        return (kP * error) + (kI * errorSum) - (kD * dError);
    }

    public void resetSum() {
        errorSum = 0;
    }
}