package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotor liftMotor;


    public Lift(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotor.class, "motor4");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void liftUp(){
        liftMotor.setPower(0.5);
    }

    public void liftDown(){
        liftMotor.setPower(-0.5);
    }

    public void stopLift(){
        liftMotor.setPower(0);
    }
}
