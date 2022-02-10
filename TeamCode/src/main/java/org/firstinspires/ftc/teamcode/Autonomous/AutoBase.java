package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class AutoBase extends LinearOpMode
{
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor leftDuck;
    protected DcMotor rightDuck;
    protected DcMotor lift;

    protected Servo lift_tilt;

    protected BNO055IMU imu;

    protected static final double driveSpeed = 0.25;
    protected static final double rotSpeed = 0.4;

    protected static final double distanceDrivekP = 0.0015;//Distance drive distance proportion

    protected static final double angleDrivekP = 0.01;//Distance drive angle proportion

    protected static final double rotkP = 0.015;//Rotational proportion

    protected static final double diameter = 3.77953;//Wheel Diameter - Inches
    protected static final double resolution = 384.5;//Tics per Revolution
    protected static final double tpi = resolution / (Math.PI * diameter);//Tics per Inches

    protected static final double liftDrivekP = 0.05;;//Lift proportion constant

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS =
            {
                    "Ball",
                    "Cube",
                    "Duck",
                    "Marker"
            };

    private static final String VUFORIA_KEY = "AeNq4qv/////AAABmZ4Kt98fREmenOsFuziFTBCDbw4UHuQ5SHg50r2Rj9m2m56FbLpOHL+B8haVsGdd/IDa7goCSOn7ai5FyOspgj241Jz+wBkD5lFinnNnCz1gZfaz42a0kjRJ1khyQqOoMiiSaG+dXh6OTt5CXwxEM0YYrR/Ogr7YdfLG7L7Y3nJo4FqVzSRvW8Xw4xsGt9PvxJNrUmvbTWAKbgBo7RvmGrUPftmF6TE5rvZTXt/IVHgfYZEvB4cO1O8YWBCZAJSLVW7TzM3hqAWvKrxCnTauL2UKOA45mDVAQT4ahsaDjJiNHjgWc4kAoYjRyVuSg+POigZBDhQ1vSMf8bazAHpfoH0LGg/z53T0oap3BOBM8Tdb";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    protected void initHardware()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        leftDuck = hardwareMap.get(DcMotor.class, "left_duck");
        rightDuck = hardwareMap.get(DcMotor.class, "right_duck");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift_tilt = hardwareMap.get(Servo.class, "lift_tilt");

        //Reverse right motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Reset Encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Don't use run to position
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Hold motor positions when idle
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Init","Motor init done!");

        //Initialize parameters for imu
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Init","Imu init done!");

        telemetry.update();
    }


    //Drive for a specific amount in inches
    protected void directionDrive(int inches, boolean strafe)
    {
        //Reset Encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Don't use run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double angle;//Current angle

        double angleTarget = getAngleDegrees();

        double distanceError = (inches * tpi) - getAveragePosition(strafe);

        double position;
        //Wait for motors to finish
        while(opModeIsActive() && Math.abs(distanceError) > 10)
        {
            angle = getAngleDegrees();

            position = getAveragePosition(strafe);

            double angleError = angleTarget - angle;

            distanceError = (inches * tpi) - position;

            if(strafe)
            {
                double speed = Range.clip(distanceError * (distanceDrivekP + 0.01), -driveSpeed, driveSpeed);

                frontLeft.setPower((angleError * angleDrivekP) - speed);
                frontRight.setPower((-angleError * angleDrivekP) + speed);
                backLeft.setPower((angleError * angleDrivekP) + speed);
                backRight.setPower((-angleError * angleDrivekP) - speed);
            }

            else
            {
                double speed = Range.clip(distanceError * distanceDrivekP, -driveSpeed, driveSpeed);

                frontLeft.setPower((angleError * angleDrivekP) + speed);
                frontRight.setPower((-angleError * angleDrivekP) + speed);
                backLeft.setPower((angleError * angleDrivekP) + speed);
                backRight.setPower((-angleError * angleDrivekP) + speed);
            }

            telemetry.addData("Current Rotation", angle);
            telemetry.addData("Average Position", position);
            telemetry.addData("Rotation Error", angleError);
            telemetry.addData("Distance Error", distanceError);
            telemetry.addData("Rotation Error Proportion", angleError * angleDrivekP);
            telemetry.addData("Distance Error Proportion", distanceError * distanceDrivekP);

            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    protected double getAveragePosition(boolean strafe)
    {
        if(strafe)
            return (-frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() - backRight.getCurrentPosition()) / 4D;

        else
            return (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4D;
    }

    protected void rotate(double degrees)
    {
        double startDeg = getAngleDegrees();

        double target = degrees + startDeg;

        double angle = getAngleDegrees();

        while(opModeIsActive() && Math.abs(target - angle) > 1)
        {
            angle = getAngleDegrees();

            double error = target - angle;

            double speed = Range.clip(error * rotkP, -rotSpeed, rotSpeed);

            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);

            telemetry.addData("Current Rotation", angle);
            telemetry.addData("Rotation Error", error);
            telemetry.addData("Motor Speed", speed);

            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    protected void moveLift(int amount)
    {
        double distanceError = amount - lift.getCurrentPosition();

        double liftPower;

        //Wait for motors to finish
        while(opModeIsActive() && Math.abs(distanceError) > 10)
        {
            distanceError = amount - lift.getCurrentPosition();

            liftPower = distanceError * liftDrivekP;

            lift.setPower(liftPower);

            telemetry.addData("Distance Error", distanceError);
            telemetry.addData("Motor Power", liftPower);

            telemetry.update();
        }

        lift.setPower(0);
    }

    protected void deliver()
    {
        lift_tilt.setPosition(0.9);

        sleep(2000);//Wait for the cube to move

        lift_tilt.setPosition(0);
    }

    protected void spinDucks()
    {
        leftDuck.setPower(0.6);
        rightDuck.setPower(-0.6);

        sleep(2000);

        leftDuck.setPower(0);
        rightDuck.setPower(0);
    }

    protected double getAngleDegrees()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    protected void initObjectDetection()
    {
        initVuforia();
        initTfod();
    }

    protected int getLiftPosFromObject()
    {//TODO Check these values
        float objectPos = detectObjects();

        if(objectPos < 500)
            return -400;//Bottom

        else if(objectPos > 500 && objectPos < 1000)
            return -700;//Middle

        else if(objectPos > 1000)
            return -880;//Top

        return -700;//Default to middle
    }

    private float detectObjects()
    {
        if(tfod != null)
        {
            tfod.activate();

            tfod.setZoom(1.25, 16.0 / 9.0);
        }

        int iterations = 0;
        while(iterations < 50)
        {
            if(tfod != null)
            {
                //getUpdatedRecognitions() will return null if no new information is available since the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if(updatedRecognitions != null)
                {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    //Step through the list of recognitions and display boundary info.
                    int i = 0;
                    for(Recognition recognition : updatedRecognitions)
                    {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("left, top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("right, bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());

                        if(recognition.getLabel().equals("Duck"))
                        {
                            tfod.shutdown();

                            telemetry.addData("Object Detection", "Found duck at " + recognition.getRight());

                            telemetry.update();

                            return recognition.getRight();
                        }

                        i++;
                    }

                    telemetry.update();

                    iterations++;
                }
            }
        }

        telemetry.addData("Warning", "No objects detected!");

        telemetry.update();

        return -1;
    }
}