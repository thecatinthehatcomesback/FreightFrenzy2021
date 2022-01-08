package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

/**
 * MainTeleOp.java
 *
 *
 * A Linear opMode class that is used as our TeleOp method for the driver controlled period.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@TeleOp(name = "MainTeleOp", group = "CatTeleOp")
public class MainTeleOp extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();


    /* Declare OpMode members. */
    CatHW_Async robot;  // Use our new mecanum async hardware


    /* Constructor */
    public MainTeleOp() {
        robot = new CatHW_Async();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        // Initialize the hardware
        robot.init(hardwareMap, this, false);

        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
         ElapsedTime delayTimer = new ElapsedTime();

        while(!opModeIsActive() && !isStopRequested()){
            if (((gamepad1.x) && delayTimer.seconds() > 0.8)) {
                delayTimer.reset();

                // Changes Alliance Sides
                CatHW_Async.isRedAlliance = !CatHW_Async.isRedAlliance;
            }
            telemetry.addData("Alliance","%s",CatHW_Async.isRedAlliance?"Red":"Blue");
            telemetry.update();
        }

        if (CatHW_Async.isRedAlliance) {
            robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // Go! (Presses PLAY)
        elapsedGameTime.time(TimeUnit.SECONDS);
        elapsedGameTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean endGame = false;
        boolean under10Sec = false;
        boolean turningMode = false;



        ElapsedTime buttontime = new ElapsedTime();
        buttontime.reset();



        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            // Driver 1 Controls:
            //--------------------------------------------------------------------------------------

            if((elapsedGameTime.time() >= 80) && (elapsedGameTime.time() <= 82)){
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }else if(elapsedGameTime.time() >= 90 && (elapsedGameTime.time() <= 92)){
                gamepad1.rumble(100);
                gamepad2.rumble(100);
            }

            if(elapsedGameTime.time() > 90 && CatHW_Async.isRedAlliance && !endGame){
                robot.lights.blink(15, RevBlinkinLedDriver.BlinkinPattern.RED,1000 );
                endGame = true;

            }else if(elapsedGameTime.time() > 90 && !CatHW_Async.isRedAlliance && ! endGame){
                robot.lights.blink(15, RevBlinkinLedDriver.BlinkinPattern.BLUE,1000 );
                endGame = true;
            }

            // Drive train speed control:
            if (gamepad1.left_bumper) {
                driveSpeed = 1.00;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.30;
            } else {
                driveSpeed = 0.70;
            }

            // Input for setDrivePowers train and sets the dead-zones:
            leftFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightFront = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;
            leftBack = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) -
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) +
                    gamepad1.left_stick_x;
            rightBack = -((Math.abs(gamepad1.right_stick_y) < 0.05) ? 0 : gamepad1.right_stick_y) +
                    ((Math.abs(gamepad1.right_stick_x) < 0.05) ? 0 : gamepad1.right_stick_x) -
                    gamepad1.left_stick_x;

            // Calculate the scale factor:
            SF = robot.drive.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront = leftFront * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack = leftBack * SF * driveSpeed;
            rightBack = rightBack * SF * driveSpeed;

            // DRIVE!!!
            if (!turningMode) {
                robot.drive.setDrivePowers(leftFront, rightFront, leftBack, rightBack);
            }

            //--------------------------------------------------------------------------------------
            // Driver 2 Controls:
            //--------------------------------------------------------------------------------------
           // if (Math.abs(gamepad1.right_trigger - (gamepad1.left_trigger)) <= 0.05) {

                if(gamepad2.left_bumper){
                    robot.jaws.setJawPower(gamepad2.right_trigger - (gamepad2.left_trigger));
                } else{
                    robot.jaws.setJawPower(gamepad2.right_trigger - (gamepad2.left_trigger * 0.3));
                }
            //} else {
            //    if(gamepad1.left_bumper){
            //        robot.jaws.setJawPower(gamepad1.right_trigger-gamepad1.left_trigger);
            //    }else{
            //        robot.jaws.setJawPower(gamepad1.right_trigger-gamepad1.left_trigger * 0.5);
            //    }
            //}



            if(gamepad2.left_bumper){
                robot.jaws.setIntakeLiftDown();
            }else if(gamepad2.right_bumper){
                robot.jaws.setIntakeLiftUp();
            }
            robot.jaws.isDone(); //will shut off intake lift when done moving

            if(robot.jaws.haveFreight() && robot.jaws.isIntakeLiftDown()){
                robot.jaws.setIntakeLiftUp();
                robot.lights.blink(1, RevBlinkinLedDriver.BlinkinPattern.GREEN,1500 );


            }
            if(gamepad2.dpad_up){
                robot.jaws.setLiftThird(.8);
            }else if(gamepad2.dpad_left){
                robot.jaws.setLiftSecond(.8);
            }else if(gamepad2.dpad_down){
                robot.jaws.setLiftBottom(.8);
            } else if(gamepad2.ps) {
                robot.jaws.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.jaws.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if(gamepad2.dpad_right){
                robot.jaws.setLiftFirst(.8);
            }
            robot.jaws.bumpLift(-gamepad2.left_stick_y);

            if(gamepad2.b){
                robot.jaws.setDumpPos(0.8);
            }else{
                robot.jaws.setDumpPos(0.3);
            }

            if(gamepad2.x){
                robot.carousel.rotateCarousel();
            }

            robot.carousel.isDone(); //will check rotation and shut it off


            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------
            telemetry.addData("Power", "LF %.2f RF %.2f LB %.2f RB %.2f", leftFront, rightFront,leftBack,rightBack);

            telemetry.addData("lift pos","Cur:%d target:%d",robot.jaws.lift.getCurrentPosition(), robot.jaws.lift.getTargetPosition());
            telemetry.addData("Intake lift", "Cur:%d Target:%d",robot.jaws.intakeLift.getCurrentPosition(),robot.jaws.intakeLift.getTargetPosition());
            telemetry.addData("Game Timer","%.2f",elapsedGameTime.time());
            telemetry.addData("color","r:%3d g:%3d b:%3d a:%3d",robot.jaws.intakeColor.red(),
                    robot.jaws.intakeColor.green(),robot.jaws.intakeColor.blue(), robot.jaws.intakeColor.alpha());

            telemetry.update();


            //dashboardTelemetry.addData("PID set","%.5f  %.5f  %.5f  %.5f",RobotConstants.LAUNCH_PID.p,RobotConstants.LAUNCH_PID.i,RobotConstants.LAUNCH_PID.d,RobotConstants.LAUNCH_PID.f);
            //dashboardTelemetry.addData("High","%4d ",2800);
            //dashboardTelemetry.addData("Low","%4d ",1800);
            dashboardTelemetry.update();
        }

        robot.eyes.stop();
    }

}