package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    private DcMotor liftMotor;
    private Servo liftServo;


    public Lift(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotor.class, "motor4");
        liftServo = hardwareMap.get(Servo.class, "lift");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftServo.setDirection(Servo.Direction.FORWARD);
    }

    public void liftUp(){
        liftMotor.setPower(1.0);
    }

    public void liftDown(){
        liftMotor.setPower(-1.0);
    }

    public void stopLift(){
        liftMotor.setPower(0);
    }

    public void score(){
        liftServo.setPosition(30);
    }
    public void intake(){
        liftServo.setPosition(180);
    }
}
