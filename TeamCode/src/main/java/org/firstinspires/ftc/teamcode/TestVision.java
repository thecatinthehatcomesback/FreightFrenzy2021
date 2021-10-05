package org.firstinspires.ftc.teamcode;/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

*/

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.CatHW_Vision.mmPerInch;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

/**
 * TestVision.java
 *
 *
 *This is a test to see if we can use the field targets to orient ourselves on the field
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="TestVision", group="CatAuto")

public class TestVision extends LinearOpMode
{

    /* Declare OpMode members. */

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isPowerShot = false;
    private OpenGLMatrix lastLocation = null;




    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this, true, true);
        robot.driveClassic.IMU_Reset();


        /*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
                robot.driveOdo.updatesThread.stop();
                return;
            }
            if (gamepad1.dpad_up && (delayTimer.seconds() > 0.8)) {
                // Increases the amount of time we wait
                timeDelay += 1;
                delayTimer.reset();
            }
            if (gamepad1.dpad_down && (delayTimer.seconds() > 0.8)) {
                // Decreases the amount of time we wait
                if (timeDelay > 0) {
                    // No such thing as negative time
                    timeDelay -= 1;
                }
                delayTimer.reset();
            }
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isRedAlliance) {
                    isRedAlliance = false;
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }
            if (gamepad1.b && (delayTimer.seconds() > 0.8)) {
                isPowerShot = !isPowerShot;
                delayTimer.reset();
            }

            //Allow the intake to run in autonomous
            robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.3));
            if (robot.jaws.getJawPower() > 0.05) {
                robot.jaws.setTransferPower(0.6);
            } else {
                robot.jaws.setTransferPower(0);
            }





            /*
             * Telemetry while waiting for PLAY:
             */

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
            if (isPowerShot) {
                telemetry.addData("Goal: ", "Power Shot");
            } else {
                telemetry.addData("Goal: ", "High Goal");
            }
            telemetry.addData("X/Y/Theta Position", "%.2f %.2f %.2f",
                    robot.driveOdo.updatesThread.positionUpdate.returnXInches(),
                    robot.driveOdo.updatesThread.positionUpdate.returnYInches(),
                    robot.driveOdo.updatesThread.positionUpdate.returnOrientation());

            boolean targetVisible = false;
            for (VuforiaTrackable trackable : robot.eyes.allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.addData("Rot (deg)", "%.1f", rotation.thirdAngle + 142);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            //telemetry.addData("Num of Rings", "%s", robot.eyes.getNumRings().toString());
            //dashboardTelemetry.addData("Num of Rings", "%s", robot.eyes.getNumRings().toString());
            //dashboardTelemetry.addData("Analysis", "%d", robot.eyes.pipeline.getAnalysis());
            dashboardTelemetry.addData("Anything",1);

            dashboardTelemetry.update();

            telemetry.update();


            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */

        }



        telemetry.addData("Before IMU","1");
        telemetry.update();
        robot.driveClassic.IMU_Init();
        telemetry.addData("after IMU",1);
        telemetry.update();

        // Time Delay:
        robot.robotWait(timeDelay);
        telemetry.addData("after delay",1);
        telemetry.update();
        //powers on launcher
        //robot.launcher.powerOn();
        //drives to position to shoot rings

        robot.driveOdo.translateDrive(-40,60,.7,0,5);

        while (robot.driveOdo.isBusy()) {
            // return if the main hardware's opMode is no longer active.
            if (!opModeIsActive()) {
                return;
            }
            logVuforia();
        }
        robot.driveOdo.translateDrive(-40,60,.7,45,5);
        while (robot.driveOdo.isBusy()) {
            // return if the main hardware's opMode is no longer active.
            if (!opModeIsActive()) {
                return;
            }
            logVuforia();
        }
        robotWait(5);
        robot.driveOdo.translateDrive(-0,60,.4,45,5);
        while (robot.driveOdo.isBusy()) {
            // return if the main hardware's opMode is no longer active.
            if (!opModeIsActive()) {
                return;
            }
            logVuforia();
        }
        robotWait(3);
        robot.driveOdo.translateDrive(-4,8,.8,0,5);
        while (robot.driveOdo.isBusy()) {
            // return if the main hardware's opMode is no longer active.
            if (!opModeIsActive()) {
                return;
            }
            logVuforia();
        }
        robot.driveOdo.updatesThread.stop();
        robot.eyes.stop();
    }
    public void logVuforia(){
        // check all the trackable targets to see which one (if any) is visible.
        boolean targetVisible = false;

        for (VuforiaTrackable trackable : robot.eyes.allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            Log.d("catbot", String.format("pos x/y/theta %.1f / %.1f / %.1f", translation.get(0)/ mmPerInch, translation.get(1) / mmPerInch, rotation.thirdAngle));


        }
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.addData("Rot (deg)", "%.1f", rotation.thirdAngle + 142);
        }
        telemetry.update();
    }
    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opModeIsActive() && (delayTimer.seconds() < seconds)) {
            idle();
            logVuforia();

        }
    }

}
