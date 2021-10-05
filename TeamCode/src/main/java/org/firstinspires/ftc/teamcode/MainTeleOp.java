package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        robot.init(hardwareMap, this, true, false);
        robot.driveClassic.IMU_Init();
        robot.driveOdo.updatesThread.positionUpdate.useIMUCorrection = true;

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
        double targetAngle = 35;
        double baseTargetAngle = 35;


        ElapsedTime buttontime = new ElapsedTime();
        buttontime.reset();

        robot.tail.openGrabber();
        robot.launcher.togglePower();


        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            // Driver 1 Controls:
            //--------------------------------------------------------------------------------------
            if(gamepad1.y && (turningMode == false)){ // turn for auto aim at power shot
                turningMode = true;
                targetAngle = baseTargetAngle - 10;
                robot.driveOdo.turn(targetAngle,5);
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            if(gamepad1.a && (turningMode == false)){  // turn for auto aim
                turningMode = true;
                targetAngle = baseTargetAngle;
                double currentTheta = robot.driveOdo.updatesThread.positionUpdate.returnOrientation();
                while((targetAngle - currentTheta) < -180){
                    targetAngle += 360;
                }
                while((targetAngle - currentTheta) > 180){
                    targetAngle -= 360;
                }
                robot.driveOdo.turn(targetAngle,5);
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            if(gamepad1.dpad_left && (turningMode == false)){ // turn left 5 degrees
                turningMode = true;
                double currentTheta = robot.driveOdo.updatesThread.positionUpdate.returnOrientation();
                targetAngle = currentTheta - 5;
                robot.driveOdo.turn(targetAngle,2);
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            }
            if(gamepad1.dpad_right && (turningMode == false)){ // turn right 5 degrees
                turningMode = true;
                double currentTheta = robot.driveOdo.updatesThread.positionUpdate.returnOrientation();
                targetAngle = currentTheta + 5;
                robot.driveOdo.turn(targetAngle,2);
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            }
            if(turningMode){
                if(robot.driveOdo.isDone()){
                    turningMode = false;
                    robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
                if(Math.abs(gamepad1.left_stick_y) > 0.5 || Math.abs(gamepad1.left_stick_x) > 0.5
                        ||Math.abs(gamepad1.right_stick_y) > 0.5 || Math.abs(gamepad1.right_stick_x) > 0.5 ){
                    turningMode = false;
                    robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
                }
            }
            if(Math.abs(targetAngle-robot.driveOdo.updatesThread.positionUpdate.returnOrientation()) > 4){
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
            if(gamepad1.ps){
                baseTargetAngle = robot.driveOdo.updatesThread.positionUpdate.returnOrientation();
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
            SF = robot.driveClassic.findScalor(leftFront, rightFront, leftBack, rightBack);
            // Set powers to each setDrivePowers motor:
            leftFront = leftFront * SF * driveSpeed;
            rightFront = rightFront * SF * driveSpeed;
            leftBack = leftBack * SF * driveSpeed;
            rightBack = rightBack * SF * driveSpeed;
                // DRIVE!!!
            if (!turningMode) {

                robot.driveClassic.setDrivePowers(leftFront, rightFront, leftBack, rightBack);
            }
            // Jaws Control:
            if (gamepad1.left_bumper) {
                robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger));
            } else {
                robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.3));
            }

            // Arm servos
            robot.jaws.leftArm.setPosition(-gamepad2.right_stick_y*.6);
            robot.jaws.rightArm.setPosition(0.75-(-gamepad2.right_stick_y*.6));





            //--------------------------------------------------------------------------------------
            // Driver 2 Controls:
            //--------------------------------------------------------------------------------------
            if(gamepad2.dpad_up && buttontime.milliseconds()>250) {
                robot.launcher.increasePower();
                buttontime.reset();
            }
            if(gamepad2.dpad_down && buttontime.milliseconds() > 250){
                robot.launcher.decreasePower();
                buttontime.reset();
            }
            if (gamepad2.x) {
                robot.launcher.presetPowerShot();
            }
            if (gamepad2.y) {
                robot.launcher.presetGoal();
            }
            if(gamepad2.guide && buttontime.milliseconds() > 250){
                robot.launcher.togglePower();
                buttontime.reset();
            }
            if(gamepad2.dpad_left) {
                robot.launcher.aimL();
            } else if(gamepad2.dpad_right) {
                robot.launcher.aimHigh();
            }
            robot.launcher.updatePower();
            if(gamepad2.b){
                robot.launcher.openLauncher();
            } else {
                robot.launcher.closeLauncher();
            }
            // Tail/Stacker lift motor controls:
            if(-gamepad2.left_stick_y>.75){
                robot.tail.setArmUp();
            } else if(-gamepad2.left_stick_y<-.75){
                robot.tail.setArmDown();
            } else if(gamepad2.left_stick_x<-.75){
                robot.tail.setArmMiddle();
            }
            robot.tail.checkMotor();

            // Open and closing the Grabber
            if (gamepad2.a && buttontime.milliseconds() > 1000) {
                robot.tail.toggleGrab();
                buttontime.reset();
            }



            // Intake controls:
            if (gamepad2.left_bumper) {
                robot.jaws.setTransferPower(-1.0);
            }else if (gamepad2.right_bumper) {
                if (gamepad2.b) {
                    robot.jaws.setTransferPower(0.6);
                } else {
                    robot.jaws.setTransferPower(1.0);
                }
            }else {
                robot.jaws.setTransferPower(0.0);
            }

            if (gamepad1.right_trigger > 0.05 && gamepad1.right_trigger > gamepad1.left_trigger) {
                if (gamepad2.b) {
                    robot.jaws.setTransferPower(0.6);
                } else {
                    robot.jaws.setTransferPower(1.0);
                }
            } else if (gamepad2.right_trigger > 0.05 && gamepad2.right_trigger > gamepad2.left_trigger) {
                if (gamepad2.b) {
                    robot.jaws.setTransferPower(0.6);
                } else {
                    robot.jaws.setTransferPower(1.0);
                }
            }



            // If driver 1 isn't using jaws, let driver 2 set Jaws Control:
            if (gamepad1.right_trigger - (gamepad1.left_trigger) == 0) {
                robot.jaws.setJawPower(gamepad2.right_trigger - (gamepad2.left_trigger * 0.3));
            }



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
            telemetry.addData("Launch RPM","%.0f", robot.launcher.getLaunchRPM());

            telemetry.addData("X Position", "%.2f", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
            telemetry.addData("Y Position", "%.2f", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
            telemetry.addData("Orientation (Degrees)", "%.2f",robot.driveOdo.updatesThread.positionUpdate.returnOrientation());
            //telemetry.addData("Encoder left right horiz", "%5d  %5d   %5d",
            //      robot.driveClassic.leftFrontMotor.getCurrentPosition(),
            //        robot.driveClassic.rightFrontMotor.getCurrentPosition(),
            //      robot.driveClassic.leftRearMotor.getCurrentPosition(),
            //    robot.driveClassic.rightRearMotor.getCurrentPosition());
            telemetry.addData("Tail P/E/T", "%.2f %d %d", robot.tail.tailMover.getPower(),robot.tail.tailMover.getCurrentPosition(), robot.tail.tailMover.getTargetPosition());

            telemetry.update();

            dashboardTelemetry.addData("set RPM target", "%.0f", robot.launcher.getLaunchRPM());
            dashboardTelemetry.addData("rpm vel","%.1f" ,robot.launcher.launcher.getVelocity()* 60 / 28);
            PIDFCoefficients cof = robot.launcher.launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            dashboardTelemetry.addData("PID running","%.5f  %.5f  %.5f  %.5f", cof.p, cof.i, cof.d, cof.f);
            dashboardTelemetry.addData("PID set","%.5f  %.5f  %.5f  %.5f",RobotConstants.LAUNCH_PID.p,RobotConstants.LAUNCH_PID.i,RobotConstants.LAUNCH_PID.d,RobotConstants.LAUNCH_PID.f);
            dashboardTelemetry.addData("High","%4d ",2800);
            dashboardTelemetry.addData("Low","%4d ",1800);
            dashboardTelemetry.update();
        }

        robot.driveOdo.updatesThread.stop();
        robot.eyes.stop();
    }

}