package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CatHW_Async.java
 *
 *
 * An "hardware" class that acts as the master in which all the other "hardware" classes run
 * through.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back.
 */
public class CatHW_Async
{
    /**
     * Attribute that is used to tell the robot through all the other classes whether it is on the
     * red or blue alliance.
     */
    public static boolean isRedAlliance = true;


    /** Local OpMode members. */
    HardwareMap hwMap = null;
    LinearOpMode opMode = null;
    private static CatHW_Async myInstance = null;

    /** Other Hardware subSystems */
    CatHW_Jaws jaws = null;
    //CatHW_Claw claw = null;
    CatHW_DriveClassic driveClassic = null;
    CatHW_DriveOdo driveOdo = null;
    CatHW_Tail tail = null;
    CatHW_Launcher launcher = null;
    CatHW_Vision eyes = null;
    CatHW_Lights lights = null;



    /* Constructor */
    public CatHW_Async() {}
    public static CatHW_Async getInstance(){
        return myInstance;
    }

    /**
     * Initialize all the standard Hardware interfaces as well as all the subsystem hardware
     * classes.
     *
     * @param ahwMap is the robot's hardware map object.
     * @param theOpMode for Linear OpMode usage.
     * @param isInitOdo in case using odometry style drive.
     * @throws InterruptedException in case of error.
     */
    public void init(HardwareMap ahwMap, LinearOpMode theOpMode, boolean isInitOdo, boolean useVuforia)
            throws InterruptedException {

        // Save a reference to hardware map and opMode
        hwMap = ahwMap;
        opMode = theOpMode;

        myInstance = this;

        // Give Telemetry for each system we begin to init:
        opMode.telemetry.addData("Initialize", "Jaws...");
        opMode.telemetry.update();
        jaws = new CatHW_Jaws(this);
        jaws.init();

        opMode.telemetry.addData("Initialize", "Tail...");
        opMode.telemetry.update();
        tail = new CatHW_Tail(this);
        tail.init();

        opMode.telemetry.addData("Initialize", "Claw...");
        opMode.telemetry.update();
        //claw = new CatHW_Claw(this);
        //claw.init();

        opMode.telemetry.addData("Initialize", "Launcher...");
        opMode.telemetry.update();
        launcher = new CatHW_Launcher(this);
        launcher.init();

        opMode.telemetry.addData("Initialize", "Lights...");
        opMode.telemetry.update();
        lights = CatHW_Lights.getInstanceAndInit(this);
        lights.init();

        if (isInitOdo) {
            opMode.telemetry.addData("Initialize", "DriveOdo...");
            opMode.telemetry.update();
            driveOdo = new CatHW_DriveOdo(this);
            driveOdo.init();
        }
        opMode.telemetry.addData("Initialize", "DriveClassic...");
        opMode.telemetry.update();
        driveClassic = new CatHW_DriveClassic(this);
        driveClassic.init();

        opMode.telemetry.addData("Initialize", "Eyes...");
        opMode.telemetry.update();
        eyes = new CatHW_Vision(this);
        eyes.initVision(hwMap, useVuforia);

        opMode.telemetry.addData("Initialize", "All Done...  BOOM!");
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Common Miscellaneous Methods:
    //----------------------------------------------------------------------------------------------
    /**
     * Method which will pause the robot's action for so many seconds.  Used for actions that could
     * either take time or when some part of the robot just needs to wait.
     *
     * @param seconds that the robot's systems will be delayed.
     */
    public void robotWait(double seconds) {
        ElapsedTime delayTimer = new ElapsedTime();
        while (opMode.opModeIsActive() && (delayTimer.seconds() < seconds)) {
            opMode.idle();
            eyes.updateVuforia();
        }
    }
}