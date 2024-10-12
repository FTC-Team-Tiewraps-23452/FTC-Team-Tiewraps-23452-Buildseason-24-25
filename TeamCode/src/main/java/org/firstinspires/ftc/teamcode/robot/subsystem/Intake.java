package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * This file is an example of how a subsystem can be defined.
 * Actual subsystems could for example be an intake, or a drivetrain.
 * In a subsystem file, the different parameters and functions are defined.
 */
public class Intake {

    //Declare motor objects
    private DcMotor intakeMotor;
    private DcMotor storeMotor;

    /**
     * This is the constructor of the subsystem
     * This is the function that will be run when the subsystem is created,
     * which happens at the beginning of an OpMode.
     * The constructor should have the same name as the class (ExampleSubsystem in this case).
     *
     * @param hardwareMap This is the input of the constructor, which will be used
     *                    to link the motors and servos in the code to the motors and servos
     *                    on the actual robot
     */
    public Intake(HardwareMap hardwareMap){
        /*
        * This lines of code links the DcMotor 'myMotor' to the port on the control/expansion hub
        * labeled "motor1"
        * This 'labeling' can be done on the Driver Station by clicking on the three dots
        * in the upper right corner and then going to 'Configure Robot'
         */
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        storeMotor = hardwareMap.get(DcMotor.class, "store");

        /*
        * Normally a DC motors runs in the clockwise direction for positive values
        * If positive values need to correspond to counter clockwise rotation,
        * for example for a drivetrain, the motor can be reversed
         */
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the motors to use the integrated encoders
         * This gives a bit more precision while controlling the motors
         */
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        storeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        storeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos can also be extracted from the hardwareMap similar to DC motors
    }

    public void setIntakeMotorIn(){
        intakeMotor.setPower(0.5);
    }
    public void setIntakeMotorOut(){
        intakeMotor.setPower(-0.5);
    }
    public void setIntakeMotorOff(){
        intakeMotor.setPower(0);
    }
    public void storeMotorStore(){
        storeMotor.setTargetPosition(50);
        storeMotor.setPower(0.5);
        storeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void storeMotorIntake(){
        storeMotor.setTargetPosition(0);
        storeMotor.setPower(0.5);
        storeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}