package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        robot.drive.IMU_Init();

        // Finished!  Now tell the driver...
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (CatHW_Async.isRedAlliance) {
            //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        } else {
            //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
        }

        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        double driveSpeed;
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;
        double SF;
        boolean alreadyStone = true;
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
            if(gamepad1.right_bumper){
                robot.jaws.setJawPower(gamepad1.right_trigger-gamepad1.left_trigger);
            }else{
                robot.jaws.setJawPower(gamepad1.right_trigger-gamepad1.left_trigger*.5);
            }

            if(gamepad1.dpad_left){
                robot.drive.horizontalDrivePower(.3);

            }else if(gamepad1.dpad_right){
                robot.drive.horizontalDrivePower(-.3);
            }







            //--------------------------------------------------------------------------------------
            // Driver 2 Controls:
            //--------------------------------------------------------------------------------------
            if (gamepad1.right_trigger - (gamepad1.left_trigger) == 0) {
                robot.jaws.setJawPower(gamepad2.right_trigger - (gamepad2.left_trigger * 0.3));
            }

            if(gamepad2.left_bumper){
                robot.jaws.setIntakeLiftDown(.8);
            }else if(gamepad2.right_bumper){
                robot.jaws.setIntakeLiftUp(.8);
            }
            robot.jaws.isDone(); //will shut off intake lift when done moving

            robot.jaws.setTransferPower(-gamepad2.right_stick_y);

            if(gamepad2.dpad_up){
                robot.jaws.setLiftThird(.5);
            }else if(gamepad2.dpad_left){
                robot.jaws.setLiftSecond(.5);
            }else if(gamepad2.dpad_down){
                robot.jaws.setLiftFirst(.5);
            } else {
                //robot.jaws.setLiftPower(0);
            }
            robot.jaws.setTransferPower(-gamepad2.left_stick_y);

            if(gamepad2.b){
                robot.jaws.setDumpPos(45);
            }else{
                robot.jaws.setDumpPos(0);
            }

            if(gamepad2.x){
                robot.carousel.rotateCarousel();
            }

            robot.carousel.isDone(); //will check rotation and shut it off


            //--------------------------------------------------------------------------------------
            // Automated Driver Control Enhancements:
            //--------------------------------------------------------------------------------------


            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------
            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);



            //telemetry.addData("Encoder left right horiz", "%5d  %5d   %5d",
            //      robot.driveClassic.leftFrontMotor.getCurrentPosition(),
            //        robot.driveClassic.rightFrontMotor.getCurrentPosition(),
            //      robot.driveClassic.leftRearMotor.getCurrentPosition(),
            //    robot.driveClassic.rightRearMotor.getCurrentPosition());

            telemetry.update();


            dashboardTelemetry.addData("PID set","%.5f  %.5f  %.5f  %.5f",RobotConstants.LAUNCH_PID.p,RobotConstants.LAUNCH_PID.i,RobotConstants.LAUNCH_PID.d,RobotConstants.LAUNCH_PID.f);
            dashboardTelemetry.addData("High","%4d ",2800);
            dashboardTelemetry.addData("Low","%4d ",1800);
            dashboardTelemetry.update();
        }

        robot.eyes.stop();
    }

}