package org.firstinspires.ftc.teamcode;

/**
 * CatThreadAsync.java
 *
 *
 * An helper class containing code to be used when a thread is needed that keeps calling the
 * waitUntilDone() method of a subsystem.
 * This is NOT an OpMode.  This class is used to spawn a wait for a subsystem to be done action.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatThreadAsync extends Thread
{
    /** Class that handles all the waitUntilDone() method calls. */
    private CatHW_Subsystem subsystem;

    /* Constructor */
    public CatThreadAsync(CatHW_Subsystem newSubsystem){
        subsystem = newSubsystem;
    }

    public void run() {
        subsystem.waitUntilDone();
    }
}