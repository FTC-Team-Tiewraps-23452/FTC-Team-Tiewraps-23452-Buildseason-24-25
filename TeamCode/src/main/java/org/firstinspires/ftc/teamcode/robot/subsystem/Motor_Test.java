package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor_Test {

    private final DcMotor motorTest;


    public Motor_Test(HardwareMap hardwareMap){
        motorTest = hardwareMap.get(DcMotor.class, "motor");
    }

    public void setMotorSpeed(double speed){
        motorTest.setPower(speed);
    }
}
