package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

/**
 * LightPattern.java
 *
 *
 * TODO:  Add Javadoc for this class.  Probably a type of data set like Point class.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class LightPattern
{
    /** Time to delay between blinks (measured in milliseconds). */
    private int delayMs;
    /** Pattern that the RevBlinkinLedDriver will present. */
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    /* Constructor */
    public LightPattern(int delayMsIn, RevBlinkinLedDriver.BlinkinPattern patternIn){
        delayMs = delayMsIn;
        pattern = patternIn;
    }

    /**
     * @param pattern that pattern will be set to.
     */
    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    /**
     * @return the amount of milliseconds the light pattern needs to wait.
     */
    public int getDelayMs() {
        return delayMs;
    }

    /**
     * @return the current assigned light pattern.
     */
    public RevBlinkinLedDriver.BlinkinPattern getPattern() {
        return pattern;
    }
}
