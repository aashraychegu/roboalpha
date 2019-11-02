package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Robo Alpha 16323 Autonomus")
public  class AutonomousRoboAlpha16323Main extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftBackMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public RevColorSensorV3 colorSensor = null;

    @Override
    public void runOpMode()
    {

        boolean ischecked= false;

        // Initializing variables
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // final int CYCLE_MS = 50;     // period of each cycle
        // final double MIN_POS = 0.0;     // Minimum rotational position
        // double INCREMENT = (double) 1 / 90;
        // double position = 1;
        // boolean isPositionChanged = false;
        // Declaring DriveTrain variables
        // double leftBackPower;
        // double rightBackPower;
        // double rightFrontPower;
        //double leftFrontPower;
        //Declaring Slide Variable
        //double slidePower;
        //declaring Speed Modifier
        //double speedMod = 0.0;

        leftBackMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        //Setting DCMotor Direction
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start
        waitForStart();
        runtime.reset();

        startMotor();
        runtime.reset();

        ischecked = isRobotOnColor();

        while(!ischecked && runtime.seconds() <=30)
        {
            //sleep(25);

            ischecked = isRobotOnColor();

        }

        stopMotor();




    } // End of RunOpfMode
    private void startMotor()
    {
        leftBackMotor.setPower(0.3);
        rightBackMotor.setPower(0.3);
        leftFrontMotor.setPower(0.3);
        rightFrontMotor.setPower(0.3);
    }
    private void stopMotor()
    {
        leftBackMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
        leftFrontMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);

    }
    private boolean isRobotOnColor()
    {
        boolean isRobotOnColorFlag = false;
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
        //telemetry.update();
        if(
                (hsvValues[0] >= 0 && hsvValues[0] <= 60)
                        && (hsvValues[1] >= 50 && hsvValues[1] <= 80)
                        && (hsvValues[2] >= 40 && hsvValues[2] <= 80)
        )
        {

            isRobotOnColorFlag = true;
        }

        if(
                (hsvValues[0] >= 190 && hsvValues[0] <= 220)
                        && (hsvValues[1] >= 0.60 && hsvValues[1] <= 0.90)
                        && (hsvValues[2] >= 0.01 && hsvValues[2] <= 0.2)
        )
        {

            isRobotOnColorFlag = true;
        }
        telemetry.addData("Flag value",isRobotOnColorFlag);
        telemetry.update();

        return isRobotOnColorFlag;
    }

}

