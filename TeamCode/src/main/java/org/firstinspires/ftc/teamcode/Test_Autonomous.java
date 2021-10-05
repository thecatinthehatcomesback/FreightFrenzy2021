package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Test_Autonomous.java
 *
 *
 * A Linear OpMode class to be place to test code both old and new.  We constantly edit this, taking
 * out and adding in code.  This is never the same at any given time.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@Autonomous(name="Test_Autonomous", group="CatTest Auto")
public class Test_Autonomous extends LinearOpMode
{
    /* Declare OpMode members. */
    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes initialize here
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;


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
        while (!opModeIsActive() ) {
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
            if (((gamepad1.dpad_left) && delayTimer.seconds() > 0.8)) {
                // Changes Alliance Sides
                if (isRedAlliance) {
                    isRedAlliance = false;
                    CatHW_Async.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    CatHW_Async.isRedAlliance = true;
                }
                delayTimer.reset();
            }



            /*
            Telemetry while waiting for PLAY:
             */
            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
            telemetry.update();

            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */
        }



        //------------------------------------------------------------------------------------------
        // Run After Play Is Pressed:
        //------------------------------------------------------------------------------------------

        /*
        Init the IMU after play so that it is not offset after remaining idle for a minute or two...
         */
        robot.driveClassic.IMU_Init();

        /* Go! */

        robot.driveOdo.translateDrive(20,35,.75,0,17,22,33,40,-5,5,2);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(20,60,.75,0,3);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,60,.75,0,2);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0,.75,0,-2,2,-5,33,0,95,3);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0,.75,90,3);
        robot.driveOdo.waitUntilDone();

        //robot.driveOdo.translateDrive(0,36, CatHW_DriveBase.DRIVE_SPEED, 0,  5);
        //robot.driveOdo.waitUntilDone();
        //robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.DRIVE_SPEED, 0,  5);
        //robot.driveOdo.waitUntilDone();
        /*robot.driveOdo.translateDrive(36,36, CatHW_DriveBase.CHILL_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,72, CatHW_DriveBase.DRIVE_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();
        robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.CHILL_SPEED, 0, .65, 5);
        robot.driveOdo.waitUntilDone();*/
        //robot.robotWait(1.5);
        //robot.driveOdo.translateDrive(0,0, CatHW_DriveBase.CREEP_SPEED, 0,  5);
        //robot.driveOdo.waitUntilDone();
        // Testing the new mecTurn
        //robot.driveClassic.mecTurn(CatHW_DriveClassic.TURN_SPEED, 170, 3.0);
    }
}
