package org.firstinspires.ftc.teamcode;/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

*/

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ododmetryTest.java
 *
 *
 * A test to see how accurate our odomerty is.
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@TeleOp(name="ododmetryTest", group="CatAuto")

public class odometryTest extends LinearOpMode
{

/* Declare OpMode members. */

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;



    @Override
    public void runOpMode() throws InterruptedException {


/*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this, true, false);



        /*
        Init Delay Option Select:
        */

        // After init is pushed but before Start we can change the delay using dpad up/down //
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
                if (isRedAlliance) {
                    isRedAlliance = false;
                    robot.isRedAlliance = false;
                } else {
                    isRedAlliance = true;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }


            /**
             * LED code:
             */

            if (robot.eyes.getNumRings() == CatHW_Vision.UltimateGoalPipeline.numRings.NONE) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            }
            if (robot.eyes.getNumRings() == CatHW_Vision.UltimateGoalPipeline.numRings.ONE) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            }
            if (robot.eyes.getNumRings() == CatHW_Vision.UltimateGoalPipeline.numRings.FOUR) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
            }


            /**
             * Telemetry while waiting for PLAY:
             */

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAlliance) {
                telemetry.addData("Alliance: ", "Red");
            } else {
                telemetry.addData("Alliance: ", "Blue");
            }
            telemetry.addData("Num of Rings", "%s",robot.eyes.getNumRings().toString());
            dashboardTelemetry.addData("Num of Rings", "%s",robot.eyes.getNumRings().toString());
            dashboardTelemetry.addData("Analysis", "%d", robot.eyes.pipeline.getAnalysis());

            dashboardTelemetry.update();

            telemetry.update();


            /**
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */

        }
        CatHW_Vision.UltimateGoalPipeline.numRings numRings = robot.eyes.getNumRings();

        /**
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */



        /**
         * Init the IMU after play so that it is not offset after
         * remaining idle for a minute or two...
         */

        robot.driveClassic.IMU_Init();

        // Time Delay:
        robot.robotWait(timeDelay);

        robot.driveOdo.quickDrive(-4,48,0.5, 0,3.0);

        for (int i = 0; i < 5; i++) {
            robot.robotWait(5);
            Log.d("catbot", String.format("Translate  Time wait    current %.2f %.2f %.1f ",
                    robot.driveOdo.updatesThread.positionUpdate.returnXInches(),
                    robot.driveOdo.updatesThread.positionUpdate.returnYInches(),
                    robot.driveOdo.updatesThread.positionUpdate.returnOrientation()
                    ));
            robot.driveOdo.quickDrive( -4,96,0.8, 0,5.0);
            robot.driveOdo.quickDrive(-52,96,0.8, 0,5.0);
            robot.driveOdo.quickDrive(-52,48,0.8, 0,5.0);
            robot.driveOdo.quickDrive( -4,48,0.8, 0,5.0);
        }
        /*
        for (int i = 0; i <= 5; i++) {
            robot.robotWait(5);
            robot.driveOdo.quickDrive( -4,96,0.3, -90,3.0);
            robot.driveOdo.quickDrive(-52,96,0.3, 0,3.0);
            robot.driveOdo.quickDrive(-52,48,0.3, 90,3.0);
            robot.driveOdo.quickDrive( -4,48,0.3, 0,3.0);
        }
*/
        robot.driveOdo.updatesThread.stop();

    }

}
