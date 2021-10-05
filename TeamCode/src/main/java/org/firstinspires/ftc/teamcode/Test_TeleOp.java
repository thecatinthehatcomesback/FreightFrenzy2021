package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Test_TeleOp.java
 *
 *
 * A Linear opMode class to be our TeleOp testing method to try and solve our problems throughout
 * the year without having to modify the main TeleOp.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@TeleOp(name="Test Tele", group="CatTest TeleOp")
public class Test_TeleOp extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime upButtonTimer = new ElapsedTime();

    /* Declare OpMode members. */
    CatHW_Async robot;

    /* Constructor */
    public Test_TeleOp() {
        robot = new CatHW_Async();
    }
    private double calcShootingAngle(){
        double goalDistanceFromWall = 36;
        double startingXCoordinate = 14;
        double fieldLength = 142;
        double startingYCoordinate = 8;
        double yPos = robot.driveOdo.updatesThread.positionUpdate.returnYInches();
        double xPos = robot.driveOdo.updatesThread.positionUpdate.returnXInches();
        double angle = Math.atan2(goalDistanceFromWall - startingXCoordinate + xPos, fieldLength - startingYCoordinate - yPos);
        return -Math.toDegrees(angle) + 23;

    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Informs driver the robot is trying to init
        telemetry.addData("Status: ", "Initializing...");
        telemetry.update();
        // Initialize the hardware
        robot.init(hardwareMap, this, true, false);
        robot.driveClassic.IMU_Init();


        // Finished!  Now tell the driver...
        telemetry.addData("Status: ", "Initialized...  BOOM!");
        telemetry.update();

        // Wait for PLAY:
        waitForStart();
        robot.driveOdo.updatesThread.positionUpdate.useIMUCorrection = true;

        if(CatHW_Async.isRedAlliance) {
            //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        } else {
            //robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE);
        }


        // Go!
        runTime.reset();
        elapsedGameTime.reset();
        upButtonTimer.reset();
//        OdometryGlobalCoordinatePosition globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.driveOdo.leftOdometry, robot.driveOdo.rightOdometry, robot.driveOdo.backOdometry, CatHW_DriveOdo.ODO_COUNTS_PER_INCH, 75);
//        Thread positionThread = new Thread(globalPositionUpdate);
//        positionThread.start();


        int power = 0;
        double driveSpeed;
        double leftFront = 0;
        double rightFront = 0;
        double leftBack = 0;
        double rightBack = 0;
        double SF;
        double grabberPosistion = 0.0;
        boolean turningMode = false;

        ElapsedTime buttontime = new ElapsedTime();
        buttontime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //--------------------------------------------------------------------------------------
            // Driver 1 Controls:
            //--------------------------------------------------------------------------------------


            if(gamepad1.a && (turningMode == false)){
                robot.driveOdo.turn(calcShootingAngle(),5);
                turningMode = true;
                robot.lights.setDefaultColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
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

                // Drive train speed control:
            if (gamepad1.left_bumper) {
                driveSpeed = 1.00;
            } else if (gamepad1.right_bumper) {
                driveSpeed = 0.30;
            } else {
                driveSpeed = 0.70;
            }

            // Input for setDrivePowers train and sets the dead-zones:
            if (!turningMode) {
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
                robot.driveClassic.setDrivePowers(leftFront, rightFront, leftBack, rightBack);
            }
            //Tests Gripper
            robot.tail.grabberServo.setPosition(grabberPosistion);
            if(gamepad1.dpad_left && upButtonTimer.milliseconds()>200){

                grabberPosistion = grabberPosistion - 0.01;
                upButtonTimer.reset();

            }
            if(gamepad1.dpad_right && upButtonTimer.milliseconds()>200){

                grabberPosistion = grabberPosistion + 0.01;
                upButtonTimer.reset();
            }

            // Jaws Control:
            if (gamepad1.left_bumper) {
                robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger));
            } else {
                robot.jaws.setJawPower(gamepad1.right_trigger - (gamepad1.left_trigger * 0.3));
            }
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



            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------

            // Tell us the odometry encoder ticks
            telemetry.addData("OdoTicks", "L/R/B  :%7d  :%7d  :%7d",
                    robot.driveOdo.updatesThread.positionUpdate.returnVerticalLeftEncoderPosition(),
                    robot.driveOdo.updatesThread.positionUpdate.returnVerticalRightEncoderPosition(),
                    robot.driveOdo.updatesThread.positionUpdate.returnHorizontalEncoderPosition() );
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", robot.driveOdo.updatesThread.positionUpdate.returnXInches());
            telemetry.addData("Y Position", robot.driveOdo.updatesThread.positionUpdate.returnYInches());
            telemetry.addData("Orientation (Degrees)", robot.driveOdo.updatesThread.positionUpdate.returnOrientation());
            //telemetry.addData("intakeSensor ", robot.jaws.hasStone());

            telemetry.addData("Left Front Power:", "%.2f", leftFront);
            telemetry.addData("Right Front Power:", "%.2f", rightFront);
            telemetry.addData("Left Back Power:", "%.2f", leftBack);
            telemetry.addData("Right Back Power:", "%.2f", rightBack);
            telemetry.addData("Grabber Servo", "%.2f",grabberPosistion);
            telemetry.addData("IMU Theta", "%.2f", robot.driveClassic.getCurrentAngle());
            telemetry.update();
        }

        // Stop the thread.
        robot.driveOdo.updatesThread.stop();
    }
}
