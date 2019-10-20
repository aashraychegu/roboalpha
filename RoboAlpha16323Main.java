package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="RoboAlpha", group="Motor Drive")
public class RALPHA16323MLSSA extends LinearOpMode {

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

    //declaring sensors: magnetic
    DigitalChannel digitalTouch;  // Hardware Device Object

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
        //declaring Speed Modifier
        double speedMod = 0.0;
        //Mapping
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
        //Sensors
        digitalTouch = hardwareMap.get(DigitalChannel.class, "magnetic_sensor");

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
        //Setting input for Digital Touch
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start
        waitForStart();
        runtime.reset();
        rightGripper.setPosition(0);
        leftGripper.setPosition(0);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Retrieve user input for DriveTrain
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.left_stick_x;
            double spin = gamepad1.right_stick_x;
            double elevation = gamepad2.left_stick_y;

            //retrieve user input for Slow Approach
            if(gamepad1.left_bumper == true)
            {
                speedMod = 0.5;

            }
            else if (gamepad1.right_bumper == true)
            {
                 speedMod = 0.3;
            }
            else {
                speedMod = 1.0;
            }
            //Magnetic Limit Switch Logic
            if (digitalTouch.getState() == false && elevation >= 0)
            {
                telemetry.addData("DigitalTouch","You can't go down. String status = NOT OK - Elevation = (%.2f)", elevation);
                telemetry.addData("DigitalTouch","Skipping setting power to Linear slide");
                slideMotor.setPower(0);
            }
            else
                {
                telemetry.addData("DigitalTouch", "You can keep going down - Elevation = (%.2f)",elevation);
                // power for liner slide
                slidePower = Range.clip(elevation, -1.0, 1.0);
                //Send Power to Slide Motor;
                slideMotor.setPower(slidePower);
            }
            telemetry.update();


            //Drivetrain Calculations
            leftBackPower = speedMod*Range.clip(drive + turn + spin, -1.0, 1.0);
            rightBackPower = speedMod*Range.clip(drive - turn - spin, -1.0, 1.0);
            rightFrontPower = speedMod*Range.clip(drive + turn - spin, -1.0, 1.0);
            leftFrontPower = speedMod*Range.clip(drive - turn + spin, -1.0, 1.0);

            // Send calculated power to wheels

            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);




            //Telemetry for Elapsed Time and DCMotor
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left back (%.2f), right back (%.2f), left front (%.2f), right front (%.2f)", leftBackPower, rightBackPower, leftFrontPower, rightFrontPower);
            telemetry.update();

            //Arm movements
            double rightStickGamePad2 = gamepad2.right_stick_y;
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
