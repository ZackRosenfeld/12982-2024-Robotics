package org.firstinspires.ftc.teamcode.AutoOp2025.funWorld;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double integralSum = 0;
    double Kp;
    double Ki;
    double Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    // Constructor, sets parameters for PID
    public PIDController(double proportional, double integral, double derivative) {
        this.Kp = proportional;
        this.Ki = integral;
        this.Kd = derivative;
    }

    // Body of the PID controller
    // Spits out a value based on the proportional, integral, and derivative constants with respect to error
    // If future me is confused, go watch a youtube video
    public double PIDControl(double target, double currentPosition) {
        double error = target - currentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return ((error * Kp) + (derivative * Kd) + (integralSum * Ki));
    }
}
