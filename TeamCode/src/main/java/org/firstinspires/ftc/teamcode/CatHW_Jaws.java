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
    public DcMotor intakeLift= null;
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

        dump = hwMap.servo.get("dump");

        liftTime = new ElapsedTime();
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
        lift.setPower(0.2);
    }
    public void setLiftFirst(double power){
        lift.setTargetPosition(100);
        lift.setPower(0.2);
    }
    public void setLiftSecond(double power){
        lift.setTargetPosition(380);
        lift.setPower(0.7);
    }
    public void setLiftThird(double power){
        lift.setTargetPosition(610);
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
        dump.setPosition(0.9);
    }
    public void unDump(){
        dump.setPosition(0.3);
    }

    // Code for the intake lift
    public void setIntakeLiftUp(){
        intakeLift.setTargetPosition(65);
        intakeLift.setPower(0.8);
    }
    public void setIntakeLiftDown(){
        intakeLift.setTargetPosition(0);
        intakeLift.setPower(-0.8);
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