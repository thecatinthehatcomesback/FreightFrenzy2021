package org.firstinspires.ftc.teamcode;


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

        lift = hwMap.dcMotor.get("lift");
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dump = hwMap.servo.get("dump");

        liftTime = new ElapsedTime();

        // Set motor modes: //
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




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

    //Lift mechanism
    public void setLiftFirst(double power){
        lift.setTargetPosition(0);
        lift.setPower(0.2);
    }
    public void setLiftSecond(double power){

        lift.setTargetPosition(300);
        lift.setPower(0.7);
    }
    public void setLiftThird(double power){

        lift.setTargetPosition(515);
        lift.setPower(0.7);
    }
    public void bumpLift(double bumpAmount) {
        if (bumpAmount > 0.5){
            lift.setTargetPosition(1 + lift.getTargetPosition());
            lift.setPower(0.4);
        }else if(bumpAmount <-0.5){
            lift.setTargetPosition(-1 + lift.getTargetPosition());
            lift.setPower(0.4);
        }
    }

    public void setLiftPower(double power){
        lift.setPower(power);
    }

    public void setDumpPos(double pos){
        dump.setPosition(pos);
    }

    public void dumpPos(){
        dump.setPosition(0.8);
    }
    public void unDump(){
        dump.setPosition(0.3);
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
        // turn off lift when it's all the way down.
        if ( (lift.getTargetPosition() == 0) && (Math.abs(lift.getCurrentPosition()) < 50)) {
            lift.setPower(0);
        }

        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = .5;
        if(liftTime.seconds()>TIMEOUT){
            intakeLift.setPower(0);
            return true;
        }
        return false;
    }
}