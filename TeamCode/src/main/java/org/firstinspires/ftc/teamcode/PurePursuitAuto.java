package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

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

@Autonomous(name="PurePursuit", group="CatAuto")

public class PurePursuitAuto extends LinearOpMode
{

/* Declare OpMode members. */

    CatHW_Async robot  = new CatHW_Async();    // All the hardware classes init here.
    private ElapsedTime delayTimer = new ElapsedTime();
    private double timeDelay;

    private ElapsedTime runningTime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Initialize the setDrivePowers system variables.  The init() methods of our hardware class
        does all the work:
         */
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.init(hardwareMap, this, false);



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
            if (((gamepad1.x) && delayTimer.seconds() > 0.5)) {
                // Changes Alliance Sides
                if (robot.isRedAlliance && !robot.isLeftAlliance) {

                    robot.isRedAlliance = true;
                    robot.isLeftAlliance = true;

                } else if(robot.isRedAlliance && robot.isLeftAlliance) {

                    robot.isLeftAlliance = true;
                    robot.isRedAlliance = false;
                } else if(!robot.isRedAlliance && robot.isLeftAlliance){

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = false;
                }else if(!robot.isRedAlliance && !robot.isLeftAlliance){

                    robot.isLeftAlliance = false;
                    robot.isRedAlliance = true;
                }
                delayTimer.reset();
            }





            /*
             * LED code:
             */
            if (CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if(CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            }else if(!CatHW_Async.isRedAlliance && !CatHW_Async.isLeftAlliance) {
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
            }else if(!CatHW_Async.isRedAlliance && CatHW_Async.isLeftAlliance){
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }



            /*
             * Telemetry while waiting for PLAY:
             */


            telemetry.addData("Delay Timer: ", timeDelay);

            if (robot.isRedAlliance && !robot.isLeftAlliance) {
                telemetry.addData("Alliance: ", "Red Warehouse");
            } else if(robot.isRedAlliance && robot.isLeftAlliance) {
                telemetry.addData("Alliance: ", "Red Carousal");
            } else if(!robot.isRedAlliance && !robot.isLeftAlliance){
                telemetry.addData("Alliance: ", "Blue Carousal");
            } else if(!robot.isRedAlliance && robot.isLeftAlliance){
                telemetry.addData("Alliance: ", "Blue Warehouse");
            }

            telemetry.addData("Duck Pos: ", "%s", robot.eyes.getDuckPos().toString());


            dashboardTelemetry.addData("Duck Pos:", "%s", robot.eyes.getDuckPos().toString());

            telemetry.addData("Analysis Right", robot.eyes.pipeline.avg1GetAnalysis());
            telemetry.addData("Analysis Middle", robot.eyes.pipeline.avg2GetAnalysis());
            telemetry.addData("Analysis Left", robot.eyes.pipeline.avg3GetAnalysis());

            robot.drive.updateOdo();
            telemetry.addData("Pos","%.3f %.3f %.3f",robot.drive.realSense.getXPos(),robot.drive.realSense.getYPos(), robot.drive.realSense.getRotation());
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
        robot.drive.realSense.resetPos();
        runningTime.reset();
        robot.robotWait(timeDelay);

        if(!robot.isRedAlliance && robot.isLeftAlliance){
            blueWarehouse();
        }else if(!robot.isRedAlliance && !robot.isLeftAlliance){
            blueCarousal();
        }else if(robot.isRedAlliance && !robot.isLeftAlliance){
            redWarehouse();


        }else if(robot.isRedAlliance && robot.isLeftAlliance){
            redLeft();

        }
}

    public void blueWarehouse(){
        ArrayList<CatType_CurvePoint> simpleDrivePath = new ArrayList<>();

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();

        simpleDrivePath.add(new CatType_CurvePoint(0, 25, -140));
        simpleDrivePath.add(new CatType_CurvePoint(19, 25, -140));

        robot.drive.pursuitDrive(simpleDrivePath, .9, 15.0, 3);

        robot.drive.waitUntilDone();

        switch(duckPos){
            case NONE:
                robot.jaws.setLiftThird(.8);
                break;
            case RIGHT:
                robot.jaws.setLiftThird(.8);

                break;
            case MIDDLE:
                robot.jaws.setLiftSecond(.8);
                break;
            case LEFT:
                robot.jaws.setLiftFirst(.8);
                break;
        }
        while (true) {

            robot.jaws.waitForLift();
            robot.jaws.dumpPos();
            robot.robotWait(.75);
            robot.jaws.unDump();
            robot.jaws.setIntakeLiftDown();
            robot.jaws.setLiftBottom(.8);

            simpleDrivePath.clear();
            simpleDrivePath.add(new CatType_CurvePoint(5, -4, -90));
            simpleDrivePath.add(new CatType_CurvePoint(-4, -5, -90));
            simpleDrivePath.add(new CatType_CurvePoint(-18, -4, -85 ));
            robot.drive.pursuitDrive(simpleDrivePath, .9, 7.0, 3);

            robot.drive.waitUntilDone();
            robot.jaws.setJawPower(.5);
            robot.drive.quickIntakeDrive(.25, 2.5);
            if (robot.jaws.haveFreight()) {
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.GREEN, 1500);

            }
            robot.jaws.setIntakeLiftUp();
            if (runningTime.seconds() > 25) {
                if (runningTime.seconds() < 29) {
                    robot.robotWait(1);
                }
                break;
            }

            robot.jaws.setIntakeLiftUp();
            simpleDrivePath.clear();
            simpleDrivePath.add(new CatType_CurvePoint(0, -5, -90));
            simpleDrivePath.add(new CatType_CurvePoint(14.5, 5, -140));
            simpleDrivePath.add(new CatType_CurvePoint(14.5, 29.5, -140));
            robot.drive.pursuitDrive(simpleDrivePath, .9, 10.0, 3);

            robot.drive.waitUntilDone();
            robot.jaws.setLiftThird(.8);
        }





    }
    public void blueCarousal(){
        ArrayList<CatType_CurvePoint> simpleDrivePath = new ArrayList<>();

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();

        simpleDrivePath.add(new CatType_CurvePoint(0, 30, 0));
        simpleDrivePath.add(new CatType_CurvePoint(-4, 44, -90));
        robot.drive.pursuitDrive(simpleDrivePath, .9, 15.0, 7);
        robot.drive.waitUntilDone();


        simpleDrivePath.clear();


        switch(duckPos){
            case NONE:
                robot.jaws.setLiftThird(.5);
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
        robot.robotWait(1.5);
        robot.jaws.dumpPos();
        robot.robotWait(2);
        robot.jaws.unDump();

        robot.jaws.setLiftBottom(.5);

        simpleDrivePath.add(new CatType_CurvePoint(26, 8, 0));
        robot.drive.pursuitDrive(simpleDrivePath, .9, 15.0, 7);
        robot.drive.waitUntilDone();

        simpleDrivePath.clear();

        robot.carousel.rotateCarousel();
        while(!robot.carousel.isDone()){
            robot.robotWait(0.1);
        }
        robot.robotWait(.5);

        robot.drive.quickDrive(26,32,90,.9,5);
    }
    public void redLeft(){
        /* Go! */
        ArrayList<CatType_CurvePoint> simpleDrivePath = new ArrayList<>();
        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        if(duckPos == CatHW_Vision.UltimateGoalPipeline.duckPosistion.MIDDLE) {
            simpleDrivePath.add(new CatType_CurvePoint(8, 0, 0));
            simpleDrivePath.add(new CatType_CurvePoint(8, 33, 0));
            simpleDrivePath.add(new CatType_CurvePoint(6, 38, -90));
            simpleDrivePath.add(new CatType_CurvePoint(10, 38, -90));
        }else {
            simpleDrivePath.add(new CatType_CurvePoint(0, 33, 0));
            simpleDrivePath.add(new CatType_CurvePoint(10, 38, -90));
        }

        robot.drive.pursuitDrive(simpleDrivePath, .9, 15.0, 7);
        robot.drive.waitUntilDone();


        simpleDrivePath.clear();

        switch(duckPos){
            case NONE:
                robot.jaws.setLiftThird(.5);

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
        robot.robotWait(1.5);
        robot.jaws.dumpPos();
        robot.robotWait(2);
        robot.jaws.unDump();
        robot.robotWait(1);
        robot.jaws.setLiftBottom(.5);
        robot.drive.quickDrive(-26,12,90,.9,5);

        robot.carousel.rotateCarousel();
        while(!robot.carousel.isDone()){
            robot.robotWait(0.1);
        }
        robot.robotWait(.5);
        robot.drive.quickDrive(-26,34,90,.9,5);
    }
    public void redWarehouse(){

        ArrayList<CatType_CurvePoint> simpleDrivePath = new ArrayList<>();

        CatHW_Vision.UltimateGoalPipeline.duckPosistion duckPos = robot.eyes.getDuckPos();
        //drive to dump position
        simpleDrivePath.clear();
        simpleDrivePath.add(new CatType_CurvePoint(0, 30.5, 135));
        simpleDrivePath.add(new CatType_CurvePoint(-6, 31.5, 135));

        robot.drive.pursuitDrive(simpleDrivePath, .9, 15.0, 3);

        robot.drive.waitUntilDone();

        switch(duckPos){
            case NONE:
                robot.jaws.setLiftThird(.8);
                break;
            case RIGHT:
                robot.jaws.setLiftThird(.8);

                break;
            case MIDDLE:
                robot.jaws.setLiftSecond(.8);
                break;
            case LEFT:
                robot.jaws.setLiftFirst(.8);
                break;
        }

        while (true) {

            robot.jaws.waitForLift();
            robot.jaws.dumpPos();
            robot.robotWait(1);
            robot.jaws.unDump();
            robot.jaws.setLiftBottom(0.5);
            robot.drive.setLooseTolerance();
            robot.jaws.setIntakeLiftDown();

            robot.drive.setLooseTolerance();

            //Drive into warehouse
            simpleDrivePath.clear();
            simpleDrivePath.add(new CatType_CurvePoint(-3, 4, 90));
            simpleDrivePath.add(new CatType_CurvePoint(20, 2, 90));
            simpleDrivePath.add(new CatType_CurvePoint(26, 4, 85));

            robot.drive.pursuitDrive(simpleDrivePath, .9, 7.0, 7);
            robot.drive.waitUntilDone();

            robot.jaws.setJawPower(.8);

            robot.drive.quickIntakeDrive(0.3, 2.5);
            if(robot.jaws.haveFreight()){
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.GREEN,1500 );

            }
            robot.jaws.setIntakeLiftUp();
            if(runningTime.seconds() > 25){
                if(runningTime.seconds() < 29){
                    robot.robotWait(1);
                }
                break;
            }
            robot.drive.setNormalTolerance();

            //drive to dump position
            simpleDrivePath.clear();
            simpleDrivePath.add(new CatType_CurvePoint(18, 3, 90));
            simpleDrivePath.add(new CatType_CurvePoint(0, 4, 90));
            simpleDrivePath.add(new CatType_CurvePoint(-7, 32.5, 135));

            robot.drive.pursuitDrive(simpleDrivePath, .9, 7.0, 7);
            robot.drive.waitUntilDone();
            robot.jaws.setJawPower(0);


            robot.jaws.setLiftThird(.8);

        }



    }
}