package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * CatHW_Tail.java
 *
 *
 * A "hardware" class containing common code accessing hardware specific to the movement and
 * rotation of the tail/stacker.
 *
 *
 * This is NOT an OpMode.  This class is used to define all the other hardware classes.
 * This hardware class assumes the following device names have been configured on the robot.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
public class CatHW_Tail extends CatHW_Subsystem
{
    /* OpMode members. */
    private static final double GRABBER_OPEN = 0.7;
    private static final double GRABBER_CLOSE = 0.25;
    private static final int ARM_DOWN = 680;
    private static final int ARM_MIDDLE = 200;
    private static final int ARM_UP = 0;
    private double timeDelay;
    private ElapsedTime delayingTimer;

    // Motors:
    public DcMotor tailMover = null;
    public Servo grabberServo   = null;

    /* local OpMode members. */
    private boolean isGrab = false;
    // Timers:
    private ElapsedTime runtime = new ElapsedTime();


    /* Constructor */
    public CatHW_Tail(CatHW_Async mainHardware) {
        super(mainHardware);
    }


    /**
     * Initialize standard Hardware interfaces.
     */
    public void init() {
        delayingTimer = new ElapsedTime();

        // Define and Initialize Motors and Servos: //
        tailMover = hwMap.dcMotor.get("tail_lift");
        grabberServo    = hwMap.servo.get("grabber_servo");
        openGrabber();

        // Set Motor and Servo Directions: //
        tailMover.setDirection(DcMotorSimple.Direction.FORWARD);

        tailMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tailMover.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int prevEncoder = tailMover.getCurrentPosition();
        tailMover.setPower(-0.15);
        mainHW.opMode.sleep (300);
        Log.d("catbot", String.format("tail lift power %.2f  current position %2d prev %2d", tailMover.getPower(), tailMover.getCurrentPosition(),prevEncoder));
        runtime.reset();
        while((Math.abs(prevEncoder - tailMover.getCurrentPosition()) > 10)&& (runtime.seconds()<3.0)){
           prevEncoder = tailMover.getCurrentPosition();
           mainHW.opMode.sleep(300);
            Log.d("catbot", String.format("tail lift power %.2f  current position %2d prev %2d", tailMover.getPower(), tailMover.getCurrentPosition(),prevEncoder));
        }
        tailMover.setPower(0.0);
        // Set Motor and Servo Modes: //
        tailMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        tailMover.setTargetPosition(0);
        tailMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        closeGrabber();
    }



    //----------------------------------------------------------------------------------------------
    // Stacker Methods:
    //----------------------------------------------------------------------------------------------


    //public void setTailPower (double tailPower) {
       // tailLift.setPower(tailPower);
    //}

    public void setArmDown (){
        timeDelay = 0;
        tailMover.setTargetPosition(ARM_DOWN);
        tailMover.setPower(0.3);
    }

    public void setArmDown (double delay){
        timeDelay = delay;
        delayingTimer.reset();

        tailMover.setTargetPosition(ARM_DOWN);
        tailMover.setPower(0.0);
    }

    public void setArmMiddle(){
        timeDelay = 0;
        tailMover.setTargetPosition(ARM_MIDDLE);
        tailMover.setPower(0.3);

    }
    public  void setArmUp(){
        timeDelay = 0;
        tailMover.setTargetPosition(ARM_UP);
        tailMover.setPower(0.3);
    }

    //Toggle Power
    public void toggleGrab () {
        if (isGrab) {
            closeGrabber();
        } else {
            openGrabber();
        }
    }

    /**
     * Closes the wobble grabber.
     */
    public void closeGrabber() {
        grabberServo.setPosition(GRABBER_CLOSE);
        isGrab = false;
    }

    /**
     * Opens the wobble grabber.
     */
    public void openGrabber() {
        grabberServo.setPosition(GRABBER_OPEN);
        isGrab = true;
    }

    public void checkMotor(){
        if(!tailMover.isBusy()){
            if(tailMover.getTargetPosition() == ARM_UP){
                tailMover.setPower(0);

            }
        }
    }



    //----------------------------------------------------------------------------------------------
    // isDone Method:
    //----------------------------------------------------------------------------------------------
    @Override
    public boolean isDone() {
        Log.d("catbot", String.format("tail lift power %.2f,", tailMover.getPower()));
        if ( (timeDelay > 0) && (delayingTimer.seconds() > timeDelay) ) {
            tailMover.setPower(0.3);
            timeDelay = 0;
        }
        /* isDone stuff for CatHW_Jaws */
        double TIMEOUT = 5.0;
        checkMotor();
        return !(tailMover.isBusy() && (runtime.seconds() < TIMEOUT));


    }
}// End of class bracket