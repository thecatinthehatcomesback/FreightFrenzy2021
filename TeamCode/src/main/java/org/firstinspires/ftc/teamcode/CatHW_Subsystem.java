package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * CatHW_Subsystem.java
 *
 *
 * A "hardware" class containing common code accessing hardware objects and processes.  It detects
 * if the subclasses are busy and can/should continue to the next step/segment of code.  This file
 * is used by CatHW_Async to run multiple processes at once.
 *
 * This is NOT an OpMode.
 *
 * This class is used to detect the different subclasses business and whether they can/should
 * continue to the next step/segment of tasks. This file is used by CatHW_Async to run multiple
 * processes at once.
 *
 *
 * @author  Team #10273, The Cat in the Hat Comes Back.
 */
public class CatHW_Subsystem
{
    /**
     * local OpMode members.
     */
    public HardwareMap hwMap = null;
    public CatHW_Async mainHW = null;

    /* Constructor */
    public CatHW_Subsystem(CatHW_Async mainHardware) {

        mainHW = mainHardware;
        hwMap = mainHW.hwMap;
    }

    /**
     * @return whether the subsystem is finished with its tasks.
     */
    public boolean isDone() {
        return true;
    }

    /**
     * @return whether the subsystem is still working on its tasks.
     */
    public boolean isBusy() {
        return !isDone();
    }


    /* Initialize standard Hardware interfaces */
    /*public void init() throws InterruptedException {

    }*/

    /**
     * Waits for the opMode to finish whatever it is doing.
     */
    public void waitUntilDone() {
        while (isBusy()) {
            // return if the main hardware's opMode is no longer active.
            if (!(mainHW.opMode.opModeIsActive())) {
                return;
            }
        }
    }

    /**
     * Waits for two different subsystems to both finish what they are doing.
     *
     * @param subOne is the first subsystem to check.
     * @param subTwo is the other subsystem to check.
     */
    public static void waitUntilDone(CatHW_Subsystem subOne, CatHW_Subsystem subTwo) {
        boolean subOneBusy = subOne.isBusy();
        boolean subTwoBusy = subTwo.isBusy();

        while (subOneBusy || subTwoBusy) {
            if (!(subOne.mainHW.opMode.opModeIsActive())) {
                return;
            }
            subOneBusy = subOne.isBusy();
            subTwoBusy = subTwo.isBusy();
        }
    }

    /**
     * Waits for three different subsystems to all finish what they are doing.
     *
     * @param subOne is the first subsystem to check.
     * @param subTwo is another subsystem to check.
     * @param subThree is the final subsystem to check.
     */
    public static void waitUntilDone(CatHW_Subsystem subOne, CatHW_Subsystem subTwo, CatHW_Subsystem subThree) {
        boolean subOneBusy = subOne.isBusy();
        boolean subTwoBusy = subTwo.isBusy();
        boolean subThreeBusy = subThree.isBusy();

        while (subOneBusy || subTwoBusy || subThreeBusy) {
            if (!(subOne.mainHW.opMode.opModeIsActive())) {
                return;
            }
            subOneBusy = subOne.isBusy();
            subTwoBusy = subTwo.isBusy();
            subThreeBusy = subThree.isBusy();
        }
    }
}