package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class CatHW_Carousel{
    private static final double wheelDiameter = 4;
    private static final double carouselDiameter = 15;
    private static final double COUNTS_PER_REVOLUTION = (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1
    private static final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (wheelDiameter * Math.PI);
    private static final double countsPerCarasouelRevolution = COUNTS_PER_INCH;

    public DcMotor CarouselMotor = null;

    public void init(){
    }
}
