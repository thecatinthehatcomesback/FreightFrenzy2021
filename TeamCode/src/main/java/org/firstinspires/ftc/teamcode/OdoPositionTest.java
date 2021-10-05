package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OdoPositionTest.java
 *
 *
 * TODO: Add Javadoc for the class...
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@Autonomous(name="Odometry Test", group="CatAuto")
@Disabled
public class OdoPositionTest extends LinearOpMode
{
    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private enum testType {bigSquare, driveAndTwist};
    private testType testToRun = testType.bigSquare;


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        robot.init(hardwareMap, this, true, false);



        //------------------------------------------------------------------------------------------
        // Init Delay Option Select:
        //------------------------------------------------------------------------------------------

        // After init is pushed but before Start we can change the delay using dpad up/down: //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive()) {
            if (this.isStopRequested()) {
                // Leave the loop if STOP is pressed
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
                if (testToRun == testType.bigSquare) {
                    testToRun = testType.driveAndTwist;
                } else {
                    testToRun = testType.bigSquare;
                }
                delayTimer.reset();
            }


            /*
            Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);
            telemetry.addData("testToRun", testToRun.toString());
            telemetry.update();

            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */
        }



        //----------------------------------------------------------------------------------------------
        // Run After Play Is Pressed:
        //----------------------------------------------------------------------------------------------

        /*
        Init the IMU after play so that it is not offset after remaining idle for a minute or two...
         */
        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);

        /* Go! */
        switch (testToRun) {
            case driveAndTwist:
                driveAndTwist();
                break;
            case bigSquare:
                bigSquare();
                break;
        }
        robot.driveOdo.updatesThread.stop();
    }

    /**
     * Run the driveAndTwist test case.
     */
    private void driveAndTwist() {
        //go to block and pick it up
        robot.driveOdo.quickDrive(0, 72, .9, 90,  4);
        //robot.driveOdo.quickDrive(0, 72, .9, 90, .4, 4);
        // robot.driveOdo.quickDrive(0, 0, .9, 90, .4, 4);
        robot.driveOdo.quickDrive(0, 0, .9, 0,  4);
        // robot.driveOdo.quickDrive(0, 72, .9, 90, .3, 4);
        //robot.driveOdo.quickDrive(0, 0, .9, 0, .3, 4);
        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
        telemetry.addData("Y Position", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
        telemetry.addData("Orientation (Degrees)", robot.driveOdo.updatesThread.positionUpdate.returnOrientation());
        telemetry.update();
        //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        robot.robotWait(5.0);
        //robot.lights.addQueue(new LightPattern(1000, RevBlinkinLedDriver.BlinkinPattern.RED));
        //robot.lights.blink(3, RevBlinkinLedDriver.BlinkinPattern.RED,250);
        robot.robotWait(10.0);
    }

    /**
     * Run the bigSquare test case.
     */
    private void bigSquare() {

        //go to block and pick it up
        robot.driveOdo.quickDrive(0, 96, .9, 0,  4);
        robot.driveOdo.quickDrive(70, 96, .9, 0,  4);
        robot.driveOdo.quickDrive(70, 0, .9, 0,  4);
        robot.driveOdo.quickDrive(0, 0, .9, 0,  4);
        robot.robotWait(1.5);
        //attemt to improve
        //robot.driveOdo.quickDrive(0, 0, .4, 0, 0, 4);
        //robot.driveOdo.quickDrive(0, 0, .4, 0, .9, 4);

        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
        telemetry.addData("Y Position", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
        telemetry.addData("Orientation (Degrees)", robot.driveOdo.updatesThread.positionUpdate.returnOrientation());
        telemetry.update();
        robot.robotWait(10.0);
    }
}