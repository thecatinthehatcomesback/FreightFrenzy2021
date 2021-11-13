package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
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
    public DcMotor transferMotor = null;
    public CRServo intakeLift= null;
    public DcMotor lift = null;
    public Servo dump = null;

    public ElapsedTime liftTime = null;




    // Timers: //


    /* Constructor */
    public CatHW_Jaws(CatHW_Async mainHardware) {
        super(mainHardware);

    }


    /* Initialize standard Hardware interfaces */
    public void init() {

        // Define and initialize motors: //
        intakeMotor = hwMap.crservo.get("intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLift = hwMap.crservo.get("intake_lift");
        intakeLift.setDirection(DcMotorSimple.Direction.FORWARD);



        transferMotor = hwMap.dcMotor.get("transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //lift = hwMap.dcMotor.get("lift");
        //lift.setDirection(DcMotorSimple.Direction.FORWARD);

        dump = hwMap.servo.get("dump");

        liftTime = new ElapsedTime();

        // Set motor modes: //
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }


    //----------------------------------------------------------------------------------------------
    // Jaw Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the power of both jaw motors.
     *
     * @param power at which the motors will spin.
     */
    public void setJawPower(double power) {
        intakeMotor.setPower(power);
    }
    public double getJawPower() {
        return intakeMotor.getPower();
    }

    /**
     * Turn off both jaws motors.
     */
    public void turnOffJaws() {
        intakeMotor.setPower(0.0);
    }

    public void setIntakeLiftPower(double power){ intakeLift.setPower(power); }

    //Lift Mecenism
    public void setLiftFirst(double power){
        final int COUNTS_PER_REVOLUTION =  (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1

        lift.setTargetPosition(COUNTS_PER_REVOLUTION*5);
    }
    public void setLiftSecond(double power){
        final int COUNTS_PER_REVOLUTION =  (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1

        lift.setTargetPosition(COUNTS_PER_REVOLUTION*10);
    }
    public void setLiftThird(double power){
        final int COUNTS_PER_REVOLUTION =  (((( 1 + ( 46 / 17))) * (1 + (46 / 17))) * 28); // Accurate for gobilda 13.7:1

        lift.setTargetPosition(COUNTS_PER_REVOLUTION*15);
    }

    public void setLiftPower(double power){
        lift.setPower(power);
    }

    public void setDumpPos(double pos){
        dump.setPosition(pos);
    }


    public void setIntakeLiftUp(double power){
        intakeLift.setPower(-power);
        liftTime.reset();

    }
    public void setIntakeLiftDown(double power){
        intakeLift.setPower(power);
        liftTime.reset();
    }

    //----------------------------------------------------------------------------------------------
    // transfer Methods:
    //----------------------------------------------------------------------------------------------

    /**
     * Set the power of both transfer motors.
     *
     * @param power at which the motors will spin.
     */
    public void setTransferPower(double power) {
        transferMotor.setPower(power);
    }


    /**
     * Turn off both Transfer motors.
     */
    public void turnOffTransfer() {
        transferMotor.setPower(0.0);
    }

    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        //Log.d("catbot", String.format(" intake power %.2f,", transferMotor.getPower()));
        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = .5;
        if(liftTime.seconds()>TIMEOUT){
            intakeLift.setPower(0);
            return true;
        }
        return false;
    }
}