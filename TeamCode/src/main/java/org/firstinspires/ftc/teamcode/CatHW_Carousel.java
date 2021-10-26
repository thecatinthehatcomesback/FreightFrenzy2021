package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CatHW_Carousel extends CatHW_Subsystem{
    public CatHW_Carousel(CatHW_Async mainHardware) {
        super(mainHardware);
    }

    private static final double wheelDiameter = 4;
    private static final double carouselDiameter = 15;
    private static final double COUNTS_PER_REVOLUTION = (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1
    private static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (wheelDiameter * Math.PI);
    private static final int countsPerCarasouelRevolution = (int) Math.round(COUNTS_PER_INCH*carouselDiameter);

    public DcMotor Carousel = null;

    public void init(){
        Carousel = hwMap.get(DcMotor.class, "carousel");
        Carousel.setDirection(DcMotorSimple.Direction.REVERSE);
        Carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void rotateCarousel(){
        Carousel.setTargetPosition(countsPerCarasouelRevolution);
        Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}
