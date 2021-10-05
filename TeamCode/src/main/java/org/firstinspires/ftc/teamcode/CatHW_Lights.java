package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

/**
 * CatHW_Lights.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the LED strings.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Lights implements Runnable
{
    /** Static variable singleInstance of type Singleton. */
    private static CatHW_Lights singleInstance = null;

    // Thread run conditions: //
    private boolean isRunning   = true;
    private int     sleepTime   = 25;
    private LightPattern defaultPattern = new LightPattern(sleepTime,
            RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);

    // The list of patterns:
    private ArrayList<LightPattern> patternList = new ArrayList<>();

    // Blinkin objects:
    private RevBlinkinLedDriver lightsRight = null;
    private RevBlinkinLedDriver lightsLeft = null;
    private RevBlinkinLedDriver.BlinkinPattern patternRight;
    private RevBlinkinLedDriver.BlinkinPattern patternLeft;


    // Hardware map:
    private HardwareMap hwMap;


    /* Constructor */
    // Private constructor restricted to this class itself.
    private CatHW_Lights(CatHW_Async mainHardware) {
        hwMap = mainHardware.hwMap;
    }

    // Static method to create instance of Singleton class.
    public static CatHW_Lights getInstanceAndInit(CatHW_Async mainHardwareIn) {
        if (singleInstance == null) {
            singleInstance = new CatHW_Lights(mainHardwareIn);
        }
        return singleInstance;
    }

    /**
     * Initializes the hardware for the Blinkin modules.
     */
    public void init() {
        // Blinkin LED stuff: //
        lightsRight         = hwMap.get(RevBlinkinLedDriver.class, "blinkyRight");
        lightsLeft          = hwMap.get(RevBlinkinLedDriver.class, "blinkyLeft");
        patternRight        = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        patternLeft         = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;


        defaultPattern = new LightPattern(sleepTime,
                RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        isRunning = true;

        // Start a new Thread for lights:
        Thread lightsThread = new Thread(this);
        lightsThread.start();
        patternList.clear();
    }



    //----------------------------------------------------------------------------------------------
    // Blinkin Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Blinks a pattern so many times for a certain amount of time.
     *
     * @param num of times to blink.
     * @param color pattern to blink.
     * @param timePerBlinkMS how long to blink.
     */
    public void blink (int num, RevBlinkinLedDriver.BlinkinPattern color, int timePerBlinkMS) {
        for (int i = 0; i < num; i++){
            addQueue(new LightPattern(timePerBlinkMS,color));
            addQueue(new LightPattern(timePerBlinkMS, RevBlinkinLedDriver.BlinkinPattern.BLACK));
        }
    }

    /**
     * Sets the LED pattern.
     *
     * @param color is the name of the Blinkin pattern.
     */
    public void setDefaultColor(RevBlinkinLedDriver.BlinkinPattern color) {
        defaultPattern.setPattern(color);
    }

    /**
     * Adds a light pattern to the end of the list.
     *
     * @param lp is the light pattern.
     */
    public void addQueue(LightPattern lp) {
        synchronized (this) {
            patternList.add(lp);
        }
    }

    /**
     * @return the light pattern's pattern and delay.
     */
    public LightPattern readQueue() {
        synchronized (this) {
            // Gets the light pattern's pattern and delay.
            if (patternList.size() > 0) {
                return patternList.remove(0);
            } else {
                return defaultPattern;
            }
        }
    }



    //----------------------------------------------------------------------------------------------
    // Run and Stop methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Continue trying different light patterns until stop() is called.
     */
    @Override
    public void run() {
        while(isRunning) {

            LightPattern current = readQueue();

            // Do stuff:
            if (current.getPattern() != patternRight){
                lightsRight.setPattern(current.getPattern());
                patternRight = current.getPattern();
            }
            if (current.getPattern() != patternLeft){
                lightsLeft.setPattern(current.getPattern());
                patternLeft = current.getPattern();
            }

            // Add delay here:
            try {
                Thread.sleep(current.getDelayMs());
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Used to set isRunning to false in order to stop the run() method.
     */
    public void stop() {
        isRunning = false;
    }
}