package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * CatHW_DriveBase.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the drive train from both the odometry and standard drive motor encoders.  This has
 * been modified and/or stripped down from the CatSingleOverallHW to be the base of all the drive
 * train classes.  This file is used by the autonomous OpModes to run multiple operations at once.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 * NOTE: All names are lower case and have underscores between words.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_DriveBase  extends CatHW_Subsystem
{
    // Wheel measurement constants:
    private static final double COUNTS_PER_REVOLUTION = 537.6; // Accurate for NeveRest Orbital 20
    private static final double WHEEL_DIAMETER_INCHES = 4.0;   // For calculating circumference
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


    /* Constructor: */
    public CatHW_DriveBase(CatHW_Async mainHardware) {
        super(mainHardware);
    }


    /**
     * Initialize standard Hardware interfaces for all drive trains.
     *
     * @throws InterruptedException in case of error.
     */
    public void init() throws InterruptedException {

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


        // Blinkin LED stuff: //
        //lights = hwMap.get(RevBlinkinLedDriver.class, "blinky");
        //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        //lights.setPattern(pattern);
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

        // Log message:
        //Log.d("catbot", String.format("Drive Power  LF: %.2f, RF: %.2f, LB: %.2f, RB: %.2f", leftFront, rightFront, leftBack, rightBack));
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
}