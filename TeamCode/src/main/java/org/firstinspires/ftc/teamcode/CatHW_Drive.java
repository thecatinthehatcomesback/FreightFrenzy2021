package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

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
public class CatHW_Drive   extends CatHW_Subsystem
{

    // Wheel measurement constants:
    private static final double COUNTS_PER_REVOLUTION = (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1
    private static final double WHEEL_DIAMETER_INCHES = 96 / 25.4;   // 96mm converted to inches
    static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);


    /* Public OpMode members. */
    // Autonomous Drive Speed constants:
    static final double HYPER_SPEED = 0.95;
    static final double DRIVE_SPEED = 0.7;
    static final double CHILL_SPEED = 0.4;
    static final double CREEP_SPEED = 0.25;
    static final double TURN_SPEED = 0.6;

    // Timer stuff:
    ElapsedTime runTime = new ElapsedTime();
    double timeout = 0;

    // Turn stuff:
    int targetAngleZ;
    int baseDelta;
    boolean clockwiseTurn;

    // isDone stuff:
    static boolean isDone;

    // The IMU sensor object:
    private static BNO055IMU imu = null;
    // State used for updating telemetry:
    private Orientation angles;

    // LED stuff:
    //public RevBlinkinLedDriver lights = null;
    //public RevBlinkinLedDriver.BlinkinPattern pattern;

    // Motors:
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    /* Enums */
    enum DRIVE_METHOD {
        vertical,
        horizontal,
        turn
    }

    enum TURN_MODE {
        SPIN,
        TANK
    }

    private DRIVE_METHOD currentMethod;


    /* Local OpMode members. */
    private LinearOpMode opMode = null;

    /* Constructor */
    public CatHW_Drive(CatHW_Async mainHardware){
        super(mainHardware);
    }


    /* Initialize standard Hardware interfaces. */
    public void init()  throws InterruptedException  {


        // Define and Initialize Motors: //
        leftFrontMotor = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor = hwMap.dcMotor.get("right_front_motor");
        leftRearMotor = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor = hwMap.dcMotor.get("right_rear_motor");

        // Define motor directions: //
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        // Define motor zero power behavior: //
        setDriveToBrake();

        // Set motor modes: //
        resetDriveEncoders();
        setDriveRunWithoutEncoders();

        // Set all motors to run at no power so that the robot doesn't move during init: //
        setDrivePowers(0, 0, 0, 0);


        // Sets enums to a default value.
        currentMethod = DRIVE_METHOD.vertical;
    }

    //----------------------------------------------------------------------------------------------
    // Driving Chassis Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Sets powers to the four drive train motors.
     *
     * @param leftFront  motor's power.
     * @param rightFront motor's power.
     * @param leftBack   motor's power.
     * @param rightBack  motor's power.
     */
    public void setDrivePowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftRearMotor.setPower(leftBack);
        rightRearMotor.setPower(rightBack);
    }

    /**
     * Set drive train motors to BRAKE.
     */
    public void setDriveToBrake() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Set drive train motors to FLOAT (coast).
     */
    public void setDriveToCoast() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Set drive train motors to STOP_AND_RESET_ENCODER.
     */
    public void resetDriveEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Set drive train motors to RUN_USING_ENCODER.
     */
    public void setDriveRunUsingEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set drive train motors to RUN_WITHOUT_ENCODER.
     */
    public void setDriveRunWithoutEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set drive train motors to RUN_TO_POSITION.
     */
    public void setDriveRunToPosition() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //----------------------------------------------------------------------------------------------
    // Mathematical operations:
    //----------------------------------------------------------------------------------------------

    /**
     * Will scale down our calculated power numbers if they are greater than 1.0.  If the values
     * were greater than 1.0, the motors would spin at their max powers.  This would limit precise
     * paths the robot could take, thus we created this method to "scale down" all the values by
     * creating a scale factor so that there is a proportional difference in all the motor powers,
     * giving the robot better mobility, especially with mecanum wheels.
     *
     * @param leftFrontValue  Prospective value for motor power that may be scaled down.
     * @param rightFrontValue Prospective value for motor power that may be scaled down.
     * @param leftBackValue   Prospective value for motor power that may be scaled down.
     * @param rightBackValue  Prospective value for motor power that may be scaled down.
     * @return what should be multiplied with all the other motor powers to get a good proportion.
     */
    public double findScalor(double leftFrontValue, double rightFrontValue,
                             double leftBackValue, double rightBackValue) {
        /*
        PLANS:

        1: Look at all motor values
        2: Find the highest absolute value (the "scalor")
        3: If the highest value is not more than 1.0, we don't need to change the values
        4: But if it is higher than 1.0, we need to find the scale to get that value down to 1.0
        5: Finally, we pass OUT the scale factor so that we can scale each motor down
         */
        double scalor = 0;
        double scaleFactor;

        double[] values;
        values = new double[4];
        values[0] = Math.abs(leftFrontValue);
        values[1] = Math.abs(rightFrontValue);
        values[2] = Math.abs(leftBackValue);
        values[3] = Math.abs(rightBackValue);

        // Find highest value:
        for (int i = 0; i + 1 < values.length; i++) {
            if (values[i] > scalor) {
                scalor = values[i];
            }
        }

        // If the highest absolute value is over 1.0, we need to get to work!  Otherwise, we done...
        if (scalor > 1.0) {
            // Get the reciprocal:
            scaleFactor = 1.0 / scalor;
        } else {
            // Set to 1 so that we don't change anything we don't have to...
            scaleFactor = 1.0;
        }

        // Now we have the scale factor!
        return scaleFactor;
        // After finding scale factor, we need to scale each motor power down by the same amount...
    }


    //----------------------------------------------------------------------------------------------
    // Driving Chassis Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * This is a simpler mecanum setDrivePowers method that drives blindly straight vertically or
     * using the color sensors to find a line.
     *
     * @param power at which the robot will travel.
     * @param distance is how far the robot will travel.
     * @param timeoutS is how much time needs to pass before the robot moves onto the next step.
     *                 This is used/useful for stall outs.
     */
    public void mecDriveVertical(double power, double distance, double timeoutS) {

        // Log message:
        Log.d("catbot", String.format(" Started setDrivePowers vert pow: %.2f, dist: %.2f," +
                " time:%.2f ", power, distance, timeoutS));


        currentMethod = DRIVE_METHOD.vertical;
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

    //----------------------------------------------------------------------------------------------
    // IMU Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Initializes the IMU unit in the first REV Expansion Hub.
     */
    public void IMU_Init() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        if(imu == null) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            //the initialize method is taking a whole second
            imu.initialize(parameters);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 250);
        }
    }
    public void IMU_Reset(){
        imu = null;
    }
    /**
     * @return the robot's current orientation.
     */
    public float getCurrentAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -angles.firstAngle;
    }

}