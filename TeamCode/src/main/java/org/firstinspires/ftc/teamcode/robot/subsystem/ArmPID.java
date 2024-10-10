package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ArmPID {
    private DcMotor arm;

    private PIDController pid;
    private double kP = 0.1;
    private double kI = 0;
    private double kD = 0;

    public ArmPID(HardwareMap hardwareMap){
        arm = hardwareMap.get(DcMotor.class, "armPID");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(kP, kI, kD);


    }

    public void setPositionWithPID(int position){
        pid.setSetpoint(position);

        double motorPosition = arm.getCurrentPosition();
        double power = pid.calculate(motorPosition);

        // Zorg ervoor dat de output beperkt blijft tussen -1 en 1.
        power = Math.max(-1, Math.min(power, 1));

        arm.setPower(power);
    }


    public void stop(){
        arm.setPower(0);
    }


}
