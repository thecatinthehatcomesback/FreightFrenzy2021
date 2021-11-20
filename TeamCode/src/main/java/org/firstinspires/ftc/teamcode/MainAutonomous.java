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
    private boolean isRedAllianceLeft = false;
    private boolean isBlueAllianceRight = false;
    private boolean isBlueAllianceLeft = false;



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
            if (CatHW_Async.isRedAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }



            /*
             * Telemetry while waiting for PLAY:
             */

            telemetry.addData("Delay Timer: ", timeDelay);

            if (isRedAllianceRight) {
                telemetry.addData("Alliance: ", "Red Hub");
            } else if(isRedAllianceLeft) {
                telemetry.addData("Alliance: ", "Red Carousal");
            } else if(isBlueAllianceRight){
                telemetry.addData("Alliance: ", "Blue Carousal");
            } else if(isBlueAllianceLeft){
                telemetry.addData("Alliance: ", "Blue Hub");
            }

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

        robot.robotWait(timeDelay);
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
        robot.drive.quickDriveVertical(.5,35,5);
        robot.drive.quickTurn(0.5,-90,5);
        robot.drive.quickDriveVertical(0.5,-3.5,5);


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
        robot.jaws.setLiftFirst(.5);

        robot.drive.quickDriveHorizontal(.5,-13,5);
        robot.drive.quickDriveVertical(.5,50,5);
        robot.drive.quickDriveVertical(.3,30,5);
        robot.drive.quickTurn(.3,-90,5);
        robot.jaws.setIntakeLiftDown(0.4);
        robot.robotWait(1);


    }
    public void blueRight(){
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        robot.drive.quickDriveVertical(.5,40,5);
        robot.drive.quickTurn(.5,80,5);
        robot.drive.quickDriveVertical(.3,-2,5);

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
        robot.robotWait(1);
        robot.jaws.unDump();

        robot.jaws.setLiftFirst(.5);

        robot.drive.quickDriveVertical(.5,35,5);
        robot.drive.quickTurn(.5,160,5);
        robot.drive.quickDriveVertical(.5,22,5);
        robot.drive.quickDriveVertical(.2,18,5);

        robot.carousel.rotateCarousel();
        while(!robot.carousel.isDone()){
            robot.robotWait(0.1);
        }
        robot.robotWait(.5);

        robot.drive.quickDriveVertical(.5,-22,5);



    }
    public void redLeft(){

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        robot.drive.quickDriveHorizontal(.5,4,5);
        robot.drive.quickDriveVertical(.2,40,5);
        robot.drive.quickTurn(.5,-90,5);
        robot.drive.quickDriveVertical(.5,-7.5,5);

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
        robot.jaws.setLiftFirst(.5);

        robot.drive.quickDriveVertical(.5,36,5);
        robot.drive.quickTurn(.5,-160,5);
        robot.drive.quickDriveVertical(.5,20,5);
        robot.drive.quickDriveVertical(.2,8.5,5);

        robot.carousel.rotateCarousel();
        while(!robot.carousel.isDone()){
            robot.robotWait(0.1);
        }
        robot.robotWait(.5);

        robot.drive.quickDriveVertical(.5,-20,5);

    }
    public void redRight(){
        telemetry.addData("Red Right","Here");
        telemetry.update();

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        robot.drive.quickDriveHorizontal(0.5,-18,5);
        robot.drive.quickDriveVertical(.5,18,5);
        robot.drive.quickTurn(0.5,170,5);
        robot.drive.quickDriveVertical(0.5,-4,5);

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
        robot.jaws.setLiftFirst(.5);

        robot.drive.quickTurn(.5,100,5);
        robot.drive.quickDriveHorizontal(0.5,4,5);
        robot.drive.quickDriveVertical(.6,50,5);
        robot.drive.quickDriveVertical(.3,30,5);


    }
}