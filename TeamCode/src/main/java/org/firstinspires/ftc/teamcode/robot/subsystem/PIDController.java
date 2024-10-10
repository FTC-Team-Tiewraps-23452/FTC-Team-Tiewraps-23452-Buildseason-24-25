package org.firstinspires.ftc.teamcode.robot.subsystem;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double setpoint = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double currentPosition) {
        double error = setpoint - currentPosition;
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative);
    }
}
