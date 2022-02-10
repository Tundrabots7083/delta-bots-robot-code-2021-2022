package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRedLeft extends AutoBase
{
    @Override
    //Run at start
    //TODO: !WARNING - CHECK BATTERY VOLTAGE!
    public void runOpMode() throws InterruptedException
    {
        initHardware();//Initialize motors
      //  initObjectDetection();//Initialize Object Detection
        //Wait for start to be pushed
        waitForStart();
        //Detect ducks
      //  int pos = getLiftPosFromObject();sleep(10000);
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
        directionDrive(25, false);
        //Strafe to carousel
        directionDrive(5, true);
        //Spin ducks
        spinDucks();
        //Drive to warehouse
        directionDrive(-60, false);
        //Strafe to warehouse
        directionDrive(12, true);
        //Park
        directionDrive(-40, false);
        //Done! - Parked in warehouse
    }
}