package org.firstinspires.ftc.teamcode.Autonomous.funWorld;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double integralSum = 0;
    double Kp;
    double Ki;
    double Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double a = 0.8; // a can be anything from 0 < a < 1
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    // Constructor, sets parameters for PID
    public PIDController(double proportional, double integral, double derivative) {
        this.Kp = proportional;
        this.Ki = integral;
        this.Kd = derivative;
    }

    public void setLastErrorInitial(double position) {
        this.lastError = position;
    }

    public void resetTimerInitial() {
        timer.reset();
    }

    // Body of the PID controller
    // Spits out a value based on the proportional, integral, and derivative constants with respect to error
    // If future me is confused, go watch a youtube video
    public double PIDControl(double target, double currentPosition) {
        double error = target - currentPosition;
        integralSum += error * timer.seconds();
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * (error - lastError);
        previousFilterEstimate = currentFilterEstimate;
        double derivative = currentFilterEstimate / timer.seconds();

        double output = ((error * Kp) + (integralSum * Ki) + (derivative * Kd));

        lastError = error;
        timer.reset();

        return output;
    }
}
