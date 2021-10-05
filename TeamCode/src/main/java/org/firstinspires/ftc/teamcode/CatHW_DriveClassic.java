package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * CatHW_DriveClassic.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the setDrivePowers train.  This is a modified or stripped down version of
 * CatSingleOverallHW to run all the drive train overall.  This file is used by the new autonomous
 * OpModes to run multiple operations at once.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * NOTE: All names are lower case and have underscores between words.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_DriveClassic extends CatHW_DriveBase
{
    /* Enums */
    enum DRIVE_METHOD {
        vertical,
        horizontal,
        turn
    }

    enum DRIVE_MODE {
        findLine,
        followWall,
        driveTilDistance,
        driveUsingGyroStraight
    }

    enum TURN_MODE {
        SPIN,
        TANK
    }

    private DRIVE_MODE currentMode;
    private DRIVE_METHOD currentMethod;


    /* Local OpMode members. */
    private LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_DriveClassic(CatHW_Async mainHardware){
        super(mainHardware);
    }


    /* Initialize standard Hardware interfaces. */
    public void init()  throws InterruptedException  {

        // Calls DriveBase's init.
        super.init();

        // Sets enums to a default value.
        currentMode   = DRIVE_MODE.driveTilDistance;
        currentMethod = DRIVE_METHOD.vertical;
    }



    //----------------------------------------------------------------------------------------------
    // Driving Chassis Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Overloading the previous method so that the DRIVE_MODE is automatically driveTilDistance.
     *
     * @param power at which the robot will travel.
     * @param distance is how far the robot will travel.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void mecDriveVertical(double power, double distance, double timeoutS) {
        mecDriveVertical(power, distance, timeoutS, DRIVE_MODE.driveTilDistance);
    }

    /**
     * This is a simpler mecanum setDrivePowers method that drives blindly straight vertically or
     * using the color sensors to find a line.
     *
     * @param power at which the robot will travel.
     * @param distance is how far the robot will travel.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     * @param driveMode is the different type of modes that the robot can drive like.
     */
    public void mecDriveVertical(double power, double distance, double timeoutS,
                                 DRIVE_MODE driveMode) {

        // Log message:
        Log.d("catbot", String.format(" Started setDrivePowers vert pow: %.2f, dist: %.2f," +
                " time:%.2f ", power, distance, timeoutS));


        currentMethod = DRIVE_METHOD.vertical;
        currentMode = driveMode;
        timeout = timeoutS;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        baseDelta = 0;
        isDone = false;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller.
            newLeftFrontTarget  = (int) (distance * COUNTS_PER_INCH);
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH);
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH);
            newRightBackTarget  = (int) (distance * COUNTS_PER_INCH);

            // Set the motors to travel towards their desired targets.
            resetDriveEncoders();
            setDriveRunToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going backwards.
            if (distance < 0) {
                power = -power;
            }

            // Due to the differences in weight on each wheel, adjust powers accordingly.
            setDrivePowers(power, power, power, power);
        }
    }

    /**
     * This is a simpler mecanum setDrivePowers method that drives blindly straight horizontally
     * (positive numbers should TRANSLATE left).
     *
     * @param power at which the robot will travel.
     * @param distance is how far the robot will drive.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void mecDriveHorizontal(double power, double distance, double timeoutS) {

        // Log message:
        Log.d("catbot", String.format(" Started setDrivePowers horizontal pow: %.2f, " +
                "dist: %.2f, time:%.2f ", power, distance, timeoutS));


        currentMethod = DRIVE_METHOD.horizontal;
        timeout = timeoutS;
        isDone = false;

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // (Multiply by sqrt of 2 to compensate)
            newLeftFrontTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightFrontTarget = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newLeftBackTarget   = (int) (distance * COUNTS_PER_INCH * Math.sqrt(2));
            newRightBackTarget  = (int) -(distance * COUNTS_PER_INCH * Math.sqrt(2));

            // Set the motors to travel towards their desired targets.
            resetDriveEncoders();
            setDriveRunToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going right.
            if (distance < 0) {
                power = -power;
            }

            // Due to the differences in weight on each wheel, adjust powers accordingly.
            setDrivePowers(power, power, power, power);
        }
    }

    /**
     * Method to TRANSLATE using the drive train encoders.
     *
     * @param power at which the robot will travel.
     * @param vectorDistance is how far the robot will travel.
     * @param vectorAng is the angle that the robot will drive at.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void advMecDrive(double power, double vectorDistance,
                            double vectorAng, double timeoutS) {
        /*
         * In this mecanum setDrivePowers method, we are trying to have the robot
         * setDrivePowers at an angle while the face of the robot remains pointed
         * ahead.
         *
         *
         * / = back left and forward right motors
         * \ = back right and forward front motors
         *
         * We add the Sin of our angle to the Cos in order to get the
         * powers for \ side of motors while we subtract Sin from Cos
         * for the / side of motors.
         */

        double modLF = Math.cos(Math.toRadians(vectorAng)) + Math.sin(Math.toRadians(vectorAng));
        double modRF = Math.cos(Math.toRadians(vectorAng)) - Math.sin(Math.toRadians(vectorAng));
        double modLB = Math.cos(Math.toRadians(vectorAng)) - Math.sin(Math.toRadians(vectorAng));
        double modRB = Math.cos(Math.toRadians(vectorAng)) + Math.sin(Math.toRadians(vectorAng));

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        boolean keepDriving = true;
        isDone = false;

        if (mainHW.opMode.opModeIsActive()) {

            // Determine new target position and adjust each one to adjust for variation of wheels.
            newLeftFrontTarget  = (int) (vectorDistance * COUNTS_PER_INCH * modLF);
            newRightFrontTarget = (int) (vectorDistance * COUNTS_PER_INCH * modRF);
            newLeftBackTarget   = (int) (vectorDistance * COUNTS_PER_INCH * modLB);
            newRightBackTarget  = (int) (vectorDistance * COUNTS_PER_INCH * modRB);

            // Set the motors to travel towards their desired targets.
            resetDriveEncoders();
            setDriveRunToPosition();
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            leftRearMotor.setTargetPosition(newLeftBackTarget);
            rightRearMotor.setTargetPosition(newRightBackTarget);

            // Reset the timeout time and start motion.
            runTime.reset();

            // Negate the power if we are going backwards.
            if (vectorDistance < 0) {
                power = -power;
            }

            // Calculate motor setDrivePowers powers after we decide direction.
            double SF = findScalor(modLF, modRF, modLB, modRB);
            modLF = modLF * SF * power;
            modRF = modRF * SF * power;
            modLB = modLB * SF * power;
            modRB = modRB * SF * power;
            // Drive:
            setDrivePowers(modLF, modRF, modLB, modRB);

            while (opMode.opModeIsActive() &&
                    (runTime.seconds() < timeoutS) &&
                    keepDriving) {

                // Find the current positions so that we can display it later.
                int leftFrontPosition  = leftFrontMotor.getCurrentPosition();
                int rightFrontPosition = rightFrontMotor.getCurrentPosition();
                int leftBackPosition   = leftRearMotor.getCurrentPosition();
                int rightBackPosition  = rightRearMotor.getCurrentPosition();

                //  Exit the method once robot stops.
                if (!leftFrontMotor.isBusy() && !rightFrontMotor.isBusy() &&
                        !leftRearMotor.isBusy() && !rightRearMotor.isBusy()) {
                    keepDriving = false;
                }

                // Display it for the driver.
                opMode.telemetry.addData("New Path",
                        "Running to :%7d :%7d :%7d :%7d",
                        newLeftFrontTarget, newRightFrontTarget,
                        newLeftBackTarget, newRightBackTarget);
                opMode.telemetry.addData("Current Path",
                        "Running at :%7d :%7d :%7d :%7d",
                        leftFrontPosition, rightFrontPosition,
                        leftBackPosition, rightBackPosition);
                opMode.telemetry.addData("Power: ", "%.3f", power);
                opMode.telemetry.addData("Time: ","%.4f seconds", runTime.seconds());
                opMode.telemetry.update();
            }

            // Stop all motion.
            setDrivePowers(0, 0, 0, 0);
        }
    }

    /**
     * Turn using IMU gyro sensors.
     *
     * @param power at which the robot will travel.
     * @param degrees that the robot needs to TURN to.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     * @throws InterruptedException in case of error.
     */
    public void mecTurn(double power, int degrees, double timeoutS) throws InterruptedException {
        mecTurn(power, degrees, timeoutS, TURN_MODE.SPIN);
    }

    /**
     * Turn using IMU gyro sensors.
     *
     * @param power used to TURN.
     * @param degrees that the robot needs to TURN to.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     * @param turnMode can be SPIN or TANK.
     *                 -SPIN will TURN with a center of rotation at the center of the robot.
     *                 -TANK will TURN with a center of rotation at the center of one side of robot.
     * @throws InterruptedException in case of error.
     */
    public void mecTurn(double power, int degrees, double timeoutS, TURN_MODE turnMode) {
        /*
        Turns counterclockwise with a negative Z angle.
        Turns clockwise with a positive Z angle.
         */

        currentMethod = DRIVE_METHOD.turn;
        timeout = timeoutS;
        isDone = false;

        // Ensure that the opMode is still active.
        if (mainHW.opMode.opModeIsActive()) {
            targetAngleZ  = degrees;
            clockwiseTurn = (getCurrentAngle() < targetAngleZ);

            // Don't use encoders.  We only use the gyro angle to TURN.
            setDriveRunWithoutEncoders();
            // reset the timeout time and start motion.
            runTime.reset();

            // Log message:
            Log.d("catbot", String.format("Start TURN...  target %d, current %d  %s",
                    targetAngleZ, getCurrentAngle(), clockwiseTurn ?"CW":"CCW"));


            // Change the power based on which angle we are turning to
            if (clockwiseTurn) {
                leftFrontMotor.setPower(power);
                leftRearMotor.setPower(power);
                if (turnMode == TURN_MODE.SPIN) {
                    rightFrontMotor.setPower(-power);
                    rightRearMotor.setPower(-power);
                } else {
                    rightFrontMotor.setPower(-power/3);
                    rightRearMotor.setPower(-power/3);
                }
            } else {
                if (turnMode == TURN_MODE.SPIN) {
                    leftFrontMotor.setPower(-power);
                    leftRearMotor.setPower(-power);
                } else {
                    leftFrontMotor.setPower(-power/3);
                    leftRearMotor.setPower(-power/3);
                }
                rightFrontMotor.setPower(power);
                rightRearMotor.setPower(power);
            }
        }
    }



    //----------------------------------------------------------------------------------------------
    // isDone Methods:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        boolean keepDriving = true;
        if ((runTime.seconds() > timeout)) {
            // Log message:
            Log.d("catbot", "Timed OUT.");
            keepDriving = false;
        }
        switch (currentMethod){
            case vertical:
                // One setDrivePowers mode that drives blindly straight
                if (currentMode == DRIVE_MODE.driveTilDistance) {

                    //  Exit the method once robot stops
                    if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                            !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {
                        keepDriving = false;
                    }
                    // Log message:
                    Log.d("catbot", String.format("DriveVert LF: %d, %d;  RF: %d, %d;" +
                                    "  LB: %d, %d;  RB %d,%d",
                            leftFrontMotor.getTargetPosition(),
                            leftFrontMotor.getCurrentPosition(),
                            rightFrontMotor.getTargetPosition(),
                            rightFrontMotor.getCurrentPosition(),
                            leftRearMotor.getTargetPosition(),
                            leftRearMotor.getCurrentPosition(),
                            rightRearMotor.getTargetPosition(),
                            rightRearMotor.getCurrentPosition()));
                }
                break;

            case horizontal:
                //  Exit the method once robot stops
                if (!leftFrontMotor.isBusy() || !rightFrontMotor.isBusy() ||
                        !leftRearMotor.isBusy() || !rightRearMotor.isBusy()) {

                    keepDriving = false;
                }
                // Log message:
                Log.d("catbot", String.format("DriveHor LF: %d, %d, %.2f;  RF: %d, %d, %.2f;" +
                                "  LB: %d, %d, %.2f;  RB %d,%d, %.2f",
                        leftFrontMotor.getTargetPosition(), leftFrontMotor.getCurrentPosition(),
                        leftFrontMotor.getPower(),
                        rightFrontMotor.getTargetPosition(), rightFrontMotor.getCurrentPosition(),
                        rightFrontMotor.getPower(),
                        leftRearMotor.getTargetPosition(), leftRearMotor.getCurrentPosition(),
                        leftRearMotor.getPower(),
                        rightRearMotor.getTargetPosition(), rightRearMotor.getCurrentPosition(),
                        rightRearMotor.getPower()));

                break;

            case turn:

                float zVal = getCurrentAngle();

                // Log message:
                Log.d("catbot", String.format("TURN  target %d, current %d  %s",
                        targetAngleZ, zVal, clockwiseTurn ? "CW": "CCW"));

                if ((zVal <= targetAngleZ) && (!clockwiseTurn)) {
                    keepDriving = false;
                }
                if ((zVal >= targetAngleZ) && (clockwiseTurn)) {
                    keepDriving = false;
                }
                break;
        }

        if (!keepDriving){
            // Stop all motion
            setDrivePowers(0, 0, 0, 0);
            isDone = true;
            return true;
        }
        return isDone;
    }

}