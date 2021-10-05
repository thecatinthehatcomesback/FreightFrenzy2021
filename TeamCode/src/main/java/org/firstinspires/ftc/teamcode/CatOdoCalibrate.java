package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * CatOdoCalibrate.java
 *
 *
 * Odometry system calibration.  Run this OpMode to generate the necessary constants to calculate
 * the robot's global position on the field.  The Global Positioning Algorithm will not function
 * and will throw an error if this program is not run first.
 *
 *
 * @author Original created by Sarthak (of Wizards.exe) on 6/1/2019.
 * @author Modified by Team #10273, The Cat in the Hat Comes Back.
 */
@TeleOp(name = "Cat Odometry Calibrate", group = "Calibration")
public class CatOdoCalibrate extends LinearOpMode
{
    /* Declare OpMode members. */
    CatHW_Async robot = new CatHW_Async();  // Use our mecanum Asynchronous hardware

    private final double PIVOT_SPEED = -0.27;

    private double horizontalTickOffset = 0;

    /*
    Text files to write the values to.  The files are stored in the robot controller under Internal
    Storage\FIRST\settings
     */
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode()  throws InterruptedException {

        // Informs driver the robot is initializing the hardware classes.
        telemetry.addData("Status", "Initializing...");
        if (!wheelBaseSeparationFile.isFile()) {
            ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(0.0));
            ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(0.0));

        }
        telemetry.update();
        robot.init(hardwareMap, this, true, false);
        robot.driveOdo.IMU_Init();

        // Finished!  Now tell the driver:
        telemetry.addData("Status", "Initialized...  BOOM!");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Begin calibration (if robot is unable to pivot at these speeds, please adjust the
        // constant at the top of the code).
        while (robot.driveOdo.getCurrentAngle() > -90 && opModeIsActive()) {
            if (robot.driveOdo.getCurrentAngle() > -60) {
                robot.driveOdo.setDrivePowers(PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, -PIVOT_SPEED);

            } else {
                robot.driveOdo.setDrivePowers(PIVOT_SPEED / 2,-PIVOT_SPEED / 2,
                        PIVOT_SPEED / 2,-PIVOT_SPEED / 2);
            }

            telemetry.addData("IMU Angle", robot.driveOdo.getCurrentAngle());
            telemetry.update();
        }

        // Stop the robot.
        robot.driveOdo.setDrivePowers(0, 0, 0, 0);
        robot.robotWait(1.0);

        // Record IMU and encoder values to calculate the constants for the global position
        // algorithm.
        double angle = robot.driveOdo.getCurrentAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder).
        Since the left encoder is also mapped to a setDrivePowers motor, the encoder value needs to
        be reversed with the negative sign in front.
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT!
         */
        double encoderDifference = Math.abs(robot.driveOdo.leftOdometry.getCurrentPosition()) +
                (Math.abs(robot.driveOdo.rightOdometry.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        /*double wheelBaseSeparation = (angle * verticalEncoderTickOffsetPerDegree) /
                (Math.PI * robot.driveOdo.ODO_COUNTS_PER_INCH);*/

        double wheelBaseSeparation = (2 * angle * verticalEncoderTickOffsetPerDegree) /
                (Math.PI * CatHW_DriveOdo.ODO_COUNTS_PER_INCH);

        // Negated this number to move the robot center to the actual center instead of behind it.
        horizontalTickOffset = robot.driveOdo.backOdometry.getCurrentPosition() /
                Math.toRadians(robot.driveOdo.getCurrentAngle());

        // Write the constants to text files.
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while (opModeIsActive()) {
            telemetry.addData("Odometry System Calibration Status",
                    "Calibration Complete");
            // Display calculated constants.
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            // Display raw values.
            telemetry.addData("IMU Angle", robot.driveOdo.getCurrentAngle());
            telemetry.addData("Vertical Left Position",
                    -robot.driveOdo.leftOdometry.getCurrentPosition());
            telemetry.addData("Vertical Right Position",
                    robot.driveOdo.rightOdometry.getCurrentPosition());
            telemetry.addData("Horizontal Position",
                    robot.driveOdo.backOdometry.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            // Update values.
            telemetry.update();
        }
    }
}
