package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * CatHW_Carousel.java
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


public class CatHW_Carousel extends CatHW_Subsystem{
    public CatHW_Carousel(CatHW_Async mainHardware) {
        super(mainHardware);
    }

    private static final double wheelDiameter = 4;
    private static final double carouselDiameter = 15;
    private static final double COUNTS_PER_REVOLUTION = (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1
    private static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (wheelDiameter * Math.PI);
    private static final int countsPerCarouselRevolution = (int) Math.round(COUNTS_PER_INCH*(carouselDiameter*Math.PI));
    private static final int turnCarousel = (int) Math.round(countsPerCarouselRevolution+(COUNTS_PER_INCH*5));

    public DcMotor Carousel = null;

    public void init(){
        Carousel = hwMap.get(DcMotor.class, "carousel");
        Carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        Carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateCarousel(){
        Carousel.setTargetPosition(turnCarousel);
        Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
