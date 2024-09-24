package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor liftMotor;

    public Lift(HardwareMap hardwareMap){

        liftMotor = hardwareMap.get(DcMotor.class, "motor1");

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        liftMotor.setTargetPosition(10);
//
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setMotorSpeed(double speed){
        liftMotor.setPower(speed);
    }
}
