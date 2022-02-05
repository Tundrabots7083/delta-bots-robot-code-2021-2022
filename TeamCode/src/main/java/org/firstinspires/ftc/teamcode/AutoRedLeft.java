package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@Autonomous
public class AutoRedLeft extends LinearOpMode
{
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor leftDuck;
    private DcMotor rightDuck;
    private DcMotor lift;

    private Servo lift_tilt;

    private BNO055IMU imu;

    public static double driveSpeed = 0.25;
    public static double rotSpeed = 0.4;

    public static double distanceDrivekP = 0.0015;//Distance drive distance proportion

    public static double angleDrivekP = 0.01;//Distance drive angle proportion

    public static double rotkP = 0.015;//Rotational proportion

    public static double diameter = 3.77953;//Wheel Diameter - Inches
    public static double resolution = 384.5;//Tics per Revolution
    public static double tpi = resolution / (Math.PI * diameter);//Tics per Inches

    public static double liftDrivekP = 0.05;;//Lift proportion constant

    @Override
    //Run at start
    //TODO: !WARNING - CHECK BATTERY VOLTAGE!
    public void runOpMode() throws InterruptedException
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());//Initialize Telemetry

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

        //Wait for start to be pushed
        waitForStart();
        //Wait 10 seconds to avoid potential collision
        sleep(10000);
        //Drive forward out of starting pos
        directionDrive(12, false);
        //Face the shipping hub
        rotate(-45);
        //Drive to shipping hub
        directionDrive(12, false);
        //TODO Move lift to proper position based off barcode - currently middle
        moveLift(-700);
        //Deliver preload
        deliver();
        //Lift goes down
        moveLift(0);
        //Turn around to face carousel
        rotate(180);
        //Drive to carousel
        directionDrive(12, false);
        //Rotate to face wall
        rotate(-45);
        //Drive to hit wall
        directionDrive(27, false);
        //Strafe to carousel
        directionDrive(5, true);
        //Spin ducks
        spinDucks();
        //Strafe to warehouse
        directionDrive(-20, true);
        //Done! - Parked in red warehouse
    }

    //Drive for a specific amount in inches
    private void directionDrive(int inches, boolean strafe)
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

    private double getAveragePosition(boolean strafe)
    {
        if(strafe)
            return (-frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() - backRight.getCurrentPosition()) / 4D;

        else
            return (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4D;
    }

    private void rotate(double degrees)
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

    private void moveLift(int amount)
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

    private void deliver()
    {
        lift_tilt.setPosition(1);

        sleep(2000);//Wait for the cube to move

        lift_tilt.setPosition(0);
    }

    private void spinDucks()
    {
        leftDuck.setPower(0.6);
        rightDuck.setPower(-0.6);

        sleep(2000);

        leftDuck.setPower(0);
        rightDuck.setPower(0);
    }

    private double getAngleDegrees()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }
}