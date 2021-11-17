package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;





import com.acmerobotics.dashboard.FtcDashboard;


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
    private boolean isRedAllianceRight = true;
    private boolean isRedAllianceLeft = true;
    private boolean isBlueAllianceRight = true;
    private boolean isBlueAllianceLeft = true;



    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this, false);
        robot.drive.IMU_Reset();
        robot.drive.IMU_Init();


        /*
        Init Delay Option Select:
         */

        // After init is pushed but before Start we can change the delay using dpad up/down //
        delayTimer.reset();
        // Runs a loop to change certain settings while we wait to start
        while (!opModeIsActive() && !isStopRequested()) {
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
                if (isRedAllianceRight) {
                    isRedAllianceRight = false;
                    isRedAllianceLeft = true;
                    isBlueAllianceRight = false;
                    isBlueAllianceLeft = false;

                    robot.isRedAllianceRight = false;
                    robot.isRedAllianceLeft = true;
                    robot.isBlueAllianceRight = false;
                    robot.isBlueAllianceLeft = false;
                    robot.isRedAlliance = true;

                } else if(isRedAllianceLeft) {
                    isRedAllianceLeft = false;
                    isRedAllianceRight = false;
                    isBlueAllianceRight = false;
                    isBlueAllianceLeft = true;

                    robot.isRedAllianceLeft = false;
                    robot.isBlueAllianceLeft = true;
                    robot.isRedAllianceRight = false;
                    robot.isBlueAllianceRight = false;

                    robot.isRedAlliance = false;

                } else if(isBlueAllianceLeft){
                    isBlueAllianceLeft = false;
                    isBlueAllianceRight = true;
                    isRedAllianceRight = false;
                    isRedAllianceLeft = false;

                    robot.isRedAllianceLeft = false;
                    robot.isBlueAllianceRight = true;
                    robot.isRedAllianceRight = false;
                    robot.isBlueAllianceLeft = false;
                    robot.isRedAlliance = false;
                }else if(isBlueAllianceRight){
                    isBlueAllianceRight = false;
                    isRedAllianceLeft = false;
                    isBlueAllianceLeft = false;
                    isRedAllianceRight = true;

                    robot.isBlueAllianceRight = false;
                    robot.isRedAllianceLeft = false;
                    robot.isRedAllianceRight = true;
                    robot.isBlueAllianceLeft = false;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }





            /*
             * LED code:
             */




            /*
             * Telemetry while waiting for PLAY:
             */

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAllianceRight) {
                telemetry.addData("Alliance: ", "Red Right");
            } else if(isRedAllianceLeft) {
                telemetry.addData("Alliance: ", "Red Left");
            } else if(isBlueAllianceRight){
                telemetry.addData("Alliance: ", "Blue Right");
            } else if(isBlueAllianceLeft){
                telemetry.addData("Alliance: ", "Blue Left");
            }
            telemetry.addData("Stuff: ","%b %b %b %b", isRedAllianceLeft, isRedAllianceRight, isBlueAllianceLeft, isBlueAllianceRight);

            telemetry.addData("Duck Pos: ", "%s", robot.eyes.getDuckPos().toString());


            dashboardTelemetry.addData("Duck Pos:", "%s", robot.eyes.getDuckPos().toString());
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


        if(isBlueAllianceLeft){
            blueLeft();
        }else if(isBlueAllianceRight){
            blueRight();
        }else if(isRedAllianceLeft){
            redLeft();

        }else if(isRedAllianceRight){
            redRight();

        }



}

    public void blueLeft(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();



    }
    public void blueRight(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();



    }
    public void redLeft(){

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        robot.drive.quickDriveHorizontal(.5,4,5);
        robot.drive.quickDriveVertical(.3,43,5);
        robot.drive.quickTurn(.5,-90,5);
        robot.drive.quickDriveVertical(.5,-6,5);

        switch(duckPos){
            case NONE:
                break;
            case RIGHT:
                robot.jaws.setLiftThird(.5);

                break;
            case MIDDLE:
                robot.jaws.setLiftSecond(.5);
                break;
            case LEFT:
                robot.jaws.setLiftFirst(.5);
                break;
        }
        robot.robotWait(1);
        robot.jaws.dumpPos();
        robot.robotWait(.5);
        robot.jaws.unDump();
        robot.robotWait(.5);

        robot.drive.quickDriveVertical(.5,35,5);
        robot.drive.quickDriveHorizontal(.5,-20,5);
        robot.drive.quickDriveHorizontal(.2,-5,5);

        robot.carousel.rotateCarousel();

        robot.drive.quickDriveHorizontal(.5,20,5);

    }
    public void redRight(){
        telemetry.addData("Red Right","Here");
        telemetry.update();

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        robot.drive.quickDriveVertical(.5,43,5);
         //TODO: lift duck to platform

        switch(duckPos){
            case NONE:
                break;
            case RIGHT:
                break;
            case MIDDLE:
                break;
            case LEFT:
                break;
        }
        robot.drive.quickTurn(.5,90,5);
        robot.drive.quickDriveHorizontal(.5,16,5);
        robot.drive.quickDriveVertical(1,58,5);


    }
}