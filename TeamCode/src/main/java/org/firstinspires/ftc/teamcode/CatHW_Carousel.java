package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
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

    private static final double wheelDiameter = 72 / 25.4;
    private static final double carouselDiameter = 15;
    private static final double COUNTS_PER_REVOLUTION = 8192;
    private static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (wheelDiameter * Math.PI);
    private static final int countsPerCarouselRevolution = (int) Math.round(COUNTS_PER_INCH*(carouselDiameter*Math.PI));
    private static final int turnCarousel = (int) Math.round(countsPerCarouselRevolution+(COUNTS_PER_INCH*5)); //extra 5 inches to make sure duck rolls off

    public CRServo Carousel = null;
    public DcMotor carouselEncoder = null;

    public void init(){
        Carousel = hwMap.get(CRServo.class, "carousel");
        Carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        carouselEncoder = hwMap.get(DcMotor.class, "transfer");

    }


    public void rotateCarousel(){
        if(Carousel.getPower()>0.5){
            return;
        }

        carouselEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(CatHW_Async.isRedAlliance){
            Carousel.setPower(-.3);

        } else {
            Carousel.setPower(.3);
        }

    }

    public void setCarouselPower(double power){
        Carousel.setPower(power);
    }

    @Override
    public boolean isDone() {
        int encoder = 0;

        if(CatHW_Async.isRedAlliance){
            encoder = -carouselEncoder.getCurrentPosition();
        } else{
            encoder = carouselEncoder.getCurrentPosition();
        }
        Log.d("catbot", String.format(" carousel encoder %d target %d, is Red alliance %d Power %.2f",
                encoder, turnCarousel, CatHW_Async.isRedAlliance?1:0, Carousel.getPower()));
        if(Carousel.getPower() == 0){
            return true;
        }
        if(encoder < (turnCarousel * 0.2)){
            if(CatHW_Async.isRedAlliance){
                Carousel.setPower(-(.3 + (encoder/(turnCarousel*.2))*.60));
            }else{
                Carousel.setPower((.3 + (encoder/(turnCarousel*.2))*.60));
            }
        }
        if(encoder > turnCarousel){
            Carousel.setPower(0);
            return true;
        }

        return false;
    }
}
