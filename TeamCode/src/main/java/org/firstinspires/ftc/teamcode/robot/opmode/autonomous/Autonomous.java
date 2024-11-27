/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;


/**
 * This file is a template for an "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * is put on the Robot Controller and executed.
 *
 * This particular OpMode contains a template to structure your code with subsystems.
 *
 */

/*
 * After the @Autonomous, the name of the Autonomous is defined which is displayed on the Driver Station
 * The group can be filled in to group different Opmodes on the phone
 * The // before @Disabled can be removed to hide the Opmode on the Driver Station
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Linear OpMode")
public class Autonomous extends LinearOpMode {

    // Declare timer to keep track of how long the program has been running
    private final ElapsedTime runtime = new ElapsedTime();

    /*
     * Declare subsystems
     * This means that we will say that certain subsystems exist and give them a name,
     * but not yet create them, this will happen in the init() function.
     */
    private IMU imu;
    private Intake intake;
    private Lift lift;
    private MecanumDrivetrain mecanumDrivetrain;

    @Override
    public void runOpMode() {
        /*
         * This code is run after the driver presses INIT, but before the actual start of the match
         */

        // Telemetry.addData is used to display variables and text on the Driver Station
        telemetry.addData("Status", "Initializing");

        /*
         * Create all the subsystems
         * Go to the folder 'subsystems' to view the subsystems, which contain more information
         */


        imu = hardwareMap.get(IMU.class, "imu");
      
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        double WHEEL_CIRCUMFERENCE = 30.1593;
        double ENCODER_RESOLUTION = 537.7;
        double TICKS_PER_SECOND = ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE;

        // Tell the driver that initialization is complete via the Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*
         * Wait for the game to start (driver presses PLAY)
         */
        waitForStart();
        runtime.reset();

        rotate(180);
//        drive(50)


    }

    private void drive(double driveDistance) {

    }

    private void rotate(double rotaionAngle) {
        imu.resetYaw();
        while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= rotaionAngle - 4 || imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= rotaionAngle + 4 && opModeIsActive()) {
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 90 && opModeIsActive()) {
                mecanumDrivetrain.mecanumDrive(0, 0, 0.05);
            } while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= 90 && opModeIsActive()) {
                mecanumDrivetrain.mecanumDrive(0, 0, -0.05);
            }
        }
        }
    }
