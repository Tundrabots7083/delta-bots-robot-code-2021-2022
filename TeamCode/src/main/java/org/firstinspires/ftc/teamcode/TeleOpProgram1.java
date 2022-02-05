package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpProgram1 extends OpMode
{
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor leftDuck;
    private DcMotor rightDuck;
    private DcMotor lift;
    private DcMotor intake;

    private Servo lift_tilt;

    public static double liftDrivekP = 0.05;;//Lift proportion constant

    private double speedMult = 0.6;//Max speed 0.6

    private double liftAmount = 0;

    @Override
    //Run once on start.
    //TODO: !WARNING - CHECK BATTERY VOLTAGE!
    public void init()
    {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        leftDuck = hardwareMap.get(DcMotor.class, "left_duck");
        rightDuck = hardwareMap.get(DcMotor.class, "right_duck");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift_tilt = hardwareMap.get(Servo.class, "lift_tilt");

        //Reverse right motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Hold motor position when idle

        resetLift();

        telemetry.addData("Init","Motor init done!");
    }

    @Override
    //Loop after init
    public void loop()
    {
        joystickStrafe(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);//Joystick movement logic
        controlLoop();//Button detection
        moveLift();//Move lift if needed
    }

    private void controlLoop()
    {
        /*
        * Controls:
        *
        * x - b : left duck spinner - right duck spinner
        * y - a : lift tilt forward - lift tilt backward
        * l - r : slow down - speed up
        * zl - zr : intake out - intake in
        * dpad up - dpad left - dpad right - dpad down : lift top - lift middle - lift bottom - lift reset
        *
        */

        //Duck Spinners
        leftDuck.setPower(gamepad1.x ? 0.6 : 0);
        rightDuck.setPower(gamepad1.b ? -0.6 : 0);

        //Lift tilt
        if(gamepad1.y)
            lift_tilt.setPosition(0.9);

        else if(gamepad1.a)
            lift_tilt.setPosition(0);

        //Lift
        if(gamepad1.dpad_up)
            liftAmount = -900;//Top

        else if(gamepad1.dpad_left)
            liftAmount = -700;//Middle

        else if(gamepad1.dpad_right)
            liftAmount = -400;//Bottom

        else if(gamepad1.dpad_down)
            liftAmount = 0;//Reset

        //Intake
        if(gamepad1.right_trigger > 0.1)
            intake.setPower(-gamepad1.right_trigger);

        else if(gamepad1.left_trigger > 0.1)
            intake.setPower(gamepad1.left_trigger);

        else
            intake.setPower(0);
    }

    private void resetLift()
    {
        //Reset Encoder
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Don't use run to position
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveLift()
    {
        double distanceError = liftAmount - lift.getCurrentPosition();

        double liftPower = 0;

        //Wait for motors to finish
        if(Math.abs(distanceError) > 10)
            liftPower = distanceError * liftDrivekP;

        lift.setPower(liftPower);

        telemetry.addData("Distance Error", distanceError);
        telemetry.addData("Motor Power", liftPower);

        telemetry.update();
    }

    private void joystickStrafe(double y1, double x1, double x2)
    {
        double turn = x2 * 0.5;

        //Convert cartesian coordinates to polar coordinates
        double headingPower = Math.hypot(x1, -y1);
        double headingAngle = Math.atan2(x1, -y1)  + Math.PI / 4;

        //2 Speeds controlled by the bumpers 0.4 - 0.6
        if(gamepad1.right_bumper)
            speedMult = 0.6;

        else if(gamepad1.left_bumper)
            speedMult = 0.4;

        double leftFrontPower = (headingPower * Math.cos(headingAngle) - turn);
        double rightFrontPower = (headingPower * Math.sin(headingAngle) + turn);
        double leftBackPower = (headingPower * Math.sin(headingAngle) - turn);
        double rightBackPower = (headingPower * Math.cos(headingAngle) + turn);

        frontLeft.setPower(leftFrontPower * speedMult);
        frontRight.setPower(rightFrontPower * speedMult);
        backLeft.setPower(leftBackPower * speedMult);
        backRight.setPower(rightBackPower * speedMult);
    }

    /*private void joystickStrafe()
    {
        //2 Speeds controlled by the bumpers 0.4 - 0.6
        if(gamepad1.right_bumper)
            speedMult = 0.6;

        else if(gamepad1.left_bumper)
            speedMult = 0.4;

        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        //Set the motor powers
        frontLeft.setPower(frontLeftPower * speedMult);
        backLeft.setPower(backLeftPower * speedMult);
        frontRight.setPower(frontRightPower * speedMult);
        backRight.setPower(backRightPower * speedMult);
    }*/
}