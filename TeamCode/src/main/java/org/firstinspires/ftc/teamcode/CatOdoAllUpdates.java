package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * CatOdoAllUpdates.java
 *
 *
 * This class spawns a separate thread to keep track the robot encoders using USB bulk reads to get
 * consistent odometry readings which helps for position and power calculations.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatOdoAllUpdates implements Runnable
{
    /** static variable singleInstance of type Singleton. */
    private static CatOdoAllUpdates singleInstance = null;

    /** Thread run condition. */
    private boolean isRunning = true;
    /** The amount of time the thread will rest before running its tasks again. */
    private int sleepTime = 25;

    private OpenGLMatrix lastLocation = null;

    /* Classes that will run off this thread. */
    CatOdoPositionUpdate positionUpdate;
    CatOdoPowerUpdate powerUpdate;

    /**
     * Stops the position update thread.
     */
    public void stop(){ isRunning = false; }

    /* Constructor */
    public CatOdoAllUpdates(HardwareMap hwMap, DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH) {
        positionUpdate = new CatOdoPositionUpdate(hwMap, verticalEncoderLeft, verticalEncoderRight, horizontalEncoder, COUNTS_PER_INCH);
        powerUpdate = new CatOdoPowerUpdate(positionUpdate);
        //positionUpdate.reverseLeftEncoder();
    }

    /**
     * TODO:  Add Javadoc.
     *
     * //@param inExpansionHubIn
     * @param verticalEncoderLeftIn
     * @param verticalEncoderRightIn
     * @param horizontalEncoderIn
     * @param COUNTS_PER_INCHIn
     * @return
     */
    public static CatOdoAllUpdates getInstanceAndInit(HardwareMap hwMap,
                                                      DcMotor verticalEncoderLeftIn,
                                                      DcMotor verticalEncoderRightIn,
                                                      DcMotor horizontalEncoderIn,
                                                      double COUNTS_PER_INCHIn) {
        if (singleInstance == null) {
            singleInstance = new CatOdoAllUpdates(hwMap, verticalEncoderLeftIn,
                    verticalEncoderRightIn, horizontalEncoderIn, COUNTS_PER_INCHIn);
        }

        return singleInstance;
    }

    /**
     * Resets the minimum power and position for the robot.
     */
    public void resetThreads() {
        powerUpdate.resetPowerToNormal();
        positionUpdate.resetPos();
        isRunning = true;
    }

    /**
     * Runs the thread.
     */
    @Override
    public void run() {
        while(isRunning) {
            positionUpdate.globalCoordinatePositionUpdate();
            powerUpdate.updatePower();
            //logVuforia();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
