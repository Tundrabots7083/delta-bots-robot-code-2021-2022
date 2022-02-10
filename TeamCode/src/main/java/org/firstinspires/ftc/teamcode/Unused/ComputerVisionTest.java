package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class ComputerVisionTest extends LinearOpMode
{
    private static final String key = "AeNq4qv/////AAABmZ4Kt98fREmenOsFuziFTBCDbw4UHuQ5SHg50r2Rj9m2m56FbLpOHL+B8haVsGdd/IDa7goCSOn7ai5FyOspgj241Jz+wBkD5lFinnNnCz1gZfaz42a0kjRJ1khyQqOoMiiSaG+dXh6OTt5CXwxEM0YYrR/Ogr7YdfLG7L7Y3nJo4FqVzSRvW8Xw4xsGt9PvxJNrUmvbTWAKbgBo7RvmGrUPftmF6TE5rvZTXt/IVHgfYZEvB4cO1O8YWBCZAJSLVW7TzM3hqAWvKrxCnTauL2UKOA45mDVAQT4ahsaDjJiNHjgWc4kAoYjRyVuSg+POigZBDhQ1vSMf8bazAHpfoH0LGg/z53T0oap3BOBM8Tdb";

    VuforiaLocalizer vuforia;
    OpenGLMatrix targetPose;
    String targetName;

    @Override public void runOpMode()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * To get an on-phone camera preview, use the code below.
         * If no camera preview is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = key;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Camera1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the trackable objects from the Assets file, and give them meaningful names
        VuforiaTrackables targetsFreightFrenzy = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        targetsFreightFrenzy.get(0).setName("Blue Storage");
        targetsFreightFrenzy.get(1).setName("Blue Alliance Wall");
        targetsFreightFrenzy.get(2).setName("Red Storage");
        targetsFreightFrenzy.get(3).setName("Red Alliance Wall");

        // Start tracking targets in the background
        targetsFreightFrenzy.activate();

        telemetry.addData(">", "Press Play to start");

        waitForStart();

        boolean targetFound = false;    // Set to true when a target is detected by Vuforia
        double  targetRange = 0;        // Distance from camera to target in Inches
        double  targetBearing = 0;        // Robot Heading, relative to target.  Positive degrees means target is to the right.

        while (opModeIsActive())
        {
            // Look for first visible target, and save its pose.
            targetFound = false;
            for (VuforiaTrackable trackable : targetsFreightFrenzy)
            {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                {
                    targetPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();

                    // if we have a target, process the "pose" to determine the position of the target relative to the robot.
                    if (targetPose != null)
                    {
                        targetFound = true;
                        targetName  = trackable.getName();
                        VectorF trans = targetPose.getTranslation();

                        // Extract the X & Y components of the offset of the target relative to the robot
                        double targetX = trans.get(0);// Image X axis
                        double targetY = trans.get(2);// Image Z axis

                        // target range is based on distance from robot position to origin (right triangle).
                        targetRange = Math.hypot(targetX, targetY);

                        // target bearing is based on angle formed between the X axis and the target range line
                        targetBearing = Math.toDegrees(Math.asin(targetX / targetRange));

                        break;  // jump out of target tracking loop if we find a target.
                    }
                }
            }

            if (targetFound)
            {
                telemetry.addData("Object", " %s", targetName);
                telemetry.addData("Range",  targetRange);
                telemetry.addData("Bearing",targetBearing);
            }

            else
            {
                telemetry.addData(":(","Target ain't found...");
            }

            telemetry.update();
        }
    }

}
