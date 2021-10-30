package org.firstinspires.ftc.teamcode;/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

*/

import android.widget.Switch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * MainAutonomous.java
 *
 *
 * A Linear OpMode class to be an autonomous method for both Blue & Red alliance sides where we pick
 * which side of the alliance bridge we start off at with gamepad1 as well as selecting alliance
 * color and whether we park under the alliance bridge closer or further from the game field wall.
 * Also will detect the position and deliver the skystone using machine vision and move the
 * foundation.
 *
 * Mec_Odo_AutonomousLevel6_Statey is written to use machine vision and SkyStone delivery to our
 * autonomous route with the help intake jaws that suck in a stone at any orientation using a
 * "touch it-own it" approach.  A servo and two motors make up TC-73/Bucky's arm and stack stones as
 * well as our team marker.

 * This autonomous is used for our State Championship(February 7-8, 2020).
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */

@Autonomous(name="MainAutonomous", group="CatAuto")

public class MainAutonomous extends LinearOpMode
{

/* Declare OpMode members. */

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;
    private boolean isRedAlliance = true;
    private boolean isPowerShot = false;



    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this, true);
        robot.drive.IMU_Reset();
        robot.drive.IMU_Init();


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
            if (gamepad1.b && (delayTimer.seconds() > 0.8)) {
                isPowerShot = !isPowerShot;
                delayTimer.reset();
            }




            /*
             * LED code:
             */

            if (robot.eyes.getDuckPos() == CatHW_Vision.UltimateGoalPipeline.duckPosistion.MIDDLE) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
            }
            if (robot.eyes.getDuckPos() == CatHW_Vision.UltimateGoalPipeline.duckPosistion.LEFT) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            }
            if (robot.eyes.getDuckPos() == CatHW_Vision.UltimateGoalPipeline.duckPosistion.RIGHT) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
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
            telemetry.addData("Num of Rings", "%s", robot.eyes.getDuckPos().toString());
            telemetry.addData("X/Y/Theta Position", "%.2f %.2f %.2f");


            dashboardTelemetry.addData("Num of Rings", "%s", robot.eyes.getDuckPos().toString());
            telemetry.addData("Analysis Right", robot.eyes.pipeline.avg1GetAnalysis());
            telemetry.addData("Analysis Middle", robot.eyes.pipeline.avg2GetAnalysis());
            telemetry.addData("Analysis Left", robot.eyes.pipeline.avg3GetAnalysis());
            dashboardTelemetry.update();

            telemetry.update();


            /*
             * We don't need a "waitForStart()" since we've been running our own
             * loop all this time so that we can make some changes.
             */

        }



        /*
         * Runs after hit start:
         * DO STUFF FOR the OPMODE!!!
         */



}
    public void blueLeft(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();

        switch (duckPos){
            case NONE:
                break;
            case LEFT:
                driveLeft();
                break;
            case MIDDLE:
                driveMiddle();
                break;
            case RIGHT:
                driveRight();
                break;
        }

    }
    public void blueRight(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();

        switch (duckPos){
            case NONE:
                break;
            case LEFT:
                driveLeft();
                break;
            case MIDDLE:
                driveMiddle();
                break;
            case RIGHT:
                driveRight();
                break;
        }

    }
    public void redLeft(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();

        switch (duckPos){
            case NONE:
                break;
            case LEFT:
                driveLeft();
                break;
            case MIDDLE:
                driveMiddle();
                break;
            case RIGHT:
                driveRight();
                break;
        }

    }
    public void redRight(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();

        switch (duckPos){
            case NONE:
                break;
            case LEFT:
                driveLeft();
                break;
            case MIDDLE:
                driveMiddle();
                break;
            case RIGHT:
                driveRight();
                break;
        }

    }
    public void driveLeft(){

    }

    public void driveMiddle(){

    }

    public void driveRight(){

    }

}