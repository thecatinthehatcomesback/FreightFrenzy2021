package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * CatHW_Jaws.java
 *
 *
 * This class containing common code accessing hardware specific to the movement of the jaws/intake.
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Jaws extends CatHW_Subsystem
{

    // Motors: //
    public CRServo intakeMotor = null;
    public DcMotor intakeLift= null;
    public DcMotor lift = null;
    public Servo dump = null;
    public Servo capLR = null;
    public Servo capVertical = null;
    public CRServo capInOut = null;
    public ColorSensor intakeColor = null;

    public ElapsedTime liftTime = null;

    public double capLRPos = 0.66;
    public double capVerticalPos = 0.89;

    public boolean halfIntakeLift = false;

    private int lastLiftEncoder = 0;

    // Timers: //

    public ElapsedTime capLRTimer = null;
    public ElapsedTime capVerticalTimer = null;

    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: //
        intakeMotor = hwMap.crservo.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeLift = hwMap.dcMotor.get("intake_lift");
        intakeLift.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLift.setTargetPosition(0);
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        lift = hwMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        capInOut = hwMap.crservo.get("in_out");
        capInOut.setDirection(DcMotorSimple.Direction.REVERSE);
        capLR = hwMap.servo.get("left_right");
        capVertical = hwMap.servo.get("up_down");

        dump = hwMap.servo.get("dump");
        intakeColor = hwMap.colorSensor.get("intake_color");
        intakeColor.enableLed(true);

        liftTime = new ElapsedTime();
        capLRTimer = new ElapsedTime();
        capVerticalTimer = new ElapsedTime();
    }


    //----------------------------------------------------------------------------------------------
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the power of intake motor.
     *
     * @param power at which the motors will spin.
     */
    public void setJawPower(double power) {
        // Max of 80% power for VEX 393 motors
        if (power > 0.8) {
            power = 0.8;
        }
        if (power < -0.8) {
            power = -0.8;
        }

        intakeMotor.setPower(power);
    }
    public double getJawPower() {
        return intakeMotor.getPower();
    }

    /**
     * Turn off intake motor.
     */
    public void turnOffJaws() {
        intakeMotor.setPower(0.0);
    }

    //Lift mechanism
    public void setLiftBottom(double power){
        lift.setTargetPosition(0);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftFirst(double power){
        lift.setTargetPosition(100);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftSecond(double power){
        lift.setTargetPosition(260);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void setLiftThird(double power){
        lift.setTargetPosition(420);
        lastLiftEncoder = -100;
        lift.setPower(power);
    }
    public void bumpLift(double bumpAmount) {
        if (bumpAmount > 0.5){
            lift.setTargetPosition(4 + lift.getTargetPosition());
            lift.setPower(0.4);
        }else if(bumpAmount <-0.5){
            lift.setTargetPosition(-4 + lift.getTargetPosition());
            lift.setPower(0.4);
        }
    }

    public void capUnloadElement(){
        capLR.setPosition(.6);
        capLRPos = .6;
        capVertical.setPosition(.84);
        capVerticalPos = .84;

    }

    public void setLiftPower(double power){
        lift.setPower(power);
    }

    //public void setDumpPos(double pos){
    //    dump.setPosition(pos);
    //}
    public void dumpPos(){
        dump.setPosition(0.8);
    }
    public void unDump()  { dump.setPosition(0.3); }

    // Code for the intake lift
    public void setIntakeLiftUp(){
        intakeLift.setTargetPosition(65);
        intakeLift.setPower(0.8);
    }
    public void setIntakeLiftHalf(){
        intakeLift.setTargetPosition(40);
        intakeLift.setPower(.8);
    }
    public void setIntakeLiftDown(){
        intakeLift.setTargetPosition(0);
        intakeLift.setPower(-0.8);
    }
    public boolean isIntakeLiftDown(){
        if(intakeLift.getTargetPosition() > 10){
            return false;
        }
        return true;
    }

    public void capLRAdjust(double adjustment){
        if(capLRTimer.seconds() > 0.01) {
            capLRPos += adjustment * Math.abs(adjustment)  * 0.015;
            if(capLRPos > 1.0){
                capLRPos = 1.0;
            }else if(capLRPos < 0){
                capLRPos = 0;
            }
            capLR.setPosition(capLRPos);
            capLRTimer.reset();
        }
    }
    public void capVerticalAdjust(double adjustment){
        if(capVerticalTimer.seconds() > 0.01) {
            capVerticalPos += adjustment * Math.abs(adjustment)  * 0.015;
            if(capVerticalPos > 1.0){
                capVerticalPos = 1.0;
            }else if(capVerticalPos < 0){
                capVerticalPos = 0;
            }
            capVertical.setPosition(capVerticalPos);
            capVerticalTimer.reset();
        }
    }
    public void setCapInOutPower(double power){
        capInOut.setPower(power);
    }

    //intake color sensor methods
    public boolean haveFreight() {
        Log.d("catbot", String.format("Have Freight r/g/b/a %4d %4d %4d %4d",
                intakeColor.red(),intakeColor.green(),intakeColor.blue(),intakeColor.alpha()));

        if(intakeColor.alpha()>2000){
            return true;
        }
        return false;
    }

    public void waitForLift(){
        while (lift.isBusy()) {
            int liftPos = lift.getCurrentPosition();
            // return if the main hardware's opMode is no longer active.
            if (!(mainHW.opMode.opModeIsActive())) {
                return;
            }
            if(lastLiftEncoder == liftPos){
                return;
            }
            lastLiftEncoder = liftPos;
            mainHW.robotWait(0.1);
        }
    }



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        //Log.d("catbot", String.format(" intake power %.2f,", transferMotor.getPower()));
        // turn off lift when it's all the way down.
        if ( (lift.getTargetPosition() == 0) && (Math.abs(lift.getCurrentPosition()) < 50)) {
            lift.setPower(0);
        }

        /* isDone stuff for CatHW_Jaws */
        if (!intakeLift.isBusy()) {
            return true;
        }
        return false;
    }
}