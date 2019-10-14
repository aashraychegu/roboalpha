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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Robo Alpha 16323 Main", group="Motor Drive")
public class RoboAlpha16323Main extends LinearOpMode {

    //Declaring the DcMotor
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor slideMotor = null;

    //Declaring Servo Motors
    Servo servoArm;
    Servo rightGripper;
    Servo leftGripper;

    @Override
    public void runOpMode() {

        // Initializing variables
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        final int CYCLE_MS = 50;     // period of each cycle
        final double MAX_POS = 1.0;     // Maximum rotational position
        final double MIN_POS = 0.0;     // Minimum rotational position
        double INCREMENT = (double) 1 / 90;
        double position = 0.5;
        double gripperPos = 0;
        boolean isPositionChanged = false;
        // Declaring DriveTrain variables
        double leftBackPower;
        double rightBackPower;
        double rightFrontPower;
        double leftFrontPower;
        //Declaring Slide Variable
        double slidePower;

        //Mapping Motors to Variables
        //DcMotors
        leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        slideMotor = hardwareMap.get(DcMotor.class, "SlideMotor");
        //Servo Motors
        servoArm = hardwareMap.get(Servo.class, "left_hand");
        rightGripper = hardwareMap.get(Servo.class, "right_gripper");
        leftGripper = hardwareMap.get(Servo.class, "left_gripper");

        //Setting DCMotor Direction
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        //Setting Servo Motor Direction
        servoArm.setDirection(Servo.Direction.FORWARD);
        leftGripper.setDirection(Servo.Direction.FORWARD);
        rightGripper.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start
        waitForStart();
        runtime.reset();
        rightGripper.setPosition(0);
        leftGripper.setPosition(0);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Retrieve user input for DriveTrain
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double spin = gamepad1.right_stick_x;
            double elevation = gamepad2.left_stick_y;

            //Drivetrain Calculations
            leftBackPower = Range.clip(drive + turn + spin, -1.0, 1.0);
            rightBackPower = Range.clip(drive - turn - spin, -1.0, 1.0);
            rightFrontPower = Range.clip(drive + turn - spin, -1.0, 1.0);
            leftFrontPower = Range.clip(drive - turn + spin, -1.0, 1.0);
            slidePower = Range.clip(elevation, -1.0, 1.0);

            // Send calculated power to wheels
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            //Send Power to Slide Motor
            slideMotor.setPower(slidePower);

            //Telemetry for Elapsed Time and DCMotor
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left back (%.2f), right back (%.2f), left front (%.2f), right front (%.2f)", leftBackPower, rightBackPower, leftFrontPower, rightFrontPower);
            telemetry.update();

            //Arm movements
            double rightStickGamePad2 = -gamepad2.right_stick_y;
            telemetry.addData("gamepad returned", "(%.2f)", rightStickGamePad2 );
            telemetry.update();

            //checks if the position has changed. If changed, set the arm to the new position.
            isPositionChanged = false;
            if (rightStickGamePad2 > 0)
            {
                // Keep stepping up until we hit the max value.
                position += INCREMENT ;
                isPositionChanged = true;
                if (position >= MAX_POS )
                {
                    position = MAX_POS;
                }
            }
            else if(rightStickGamePad2 < 0)
            {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT ;
                isPositionChanged = true;
                if (position <= MIN_POS )
                {
                    position = MIN_POS;
                }
            }

            if(isPositionChanged)
            {
                servoArm.setPosition(position);
                telemetry.addData("setting position ", "(%.2f)", position);
                telemetry.update();
            }

            //Grabber Movement.
            if (gamepad2.y && gamepad2.a)
            {
                telemetry.addData("DRIVER 2: Invalid - Don't press both the gripper buttons!!!!!", "0");
                telemetry.update();
            }
            else if (gamepad2.y)
            {
                gripperPos = 1;
                rightGripper.setPosition(gripperPos);
                leftGripper.setPosition(gripperPos);
                telemetry.addData("DRIVER 2: Grippers have been lowered -(%.2f)",gripperPos);
                telemetry.update();
            }
            else if (gamepad2.a)
            {
                gripperPos = 0;
                rightGripper.setPosition(gripperPos);
                leftGripper.setPosition(gripperPos);
                telemetry.addData("DRIVER 2: Grippers have been raised -(%.2f)",gripperPos);
                telemetry.update();
            }
        }
    }
}
