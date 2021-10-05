package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TestMotorTeleOp.java
 *
 *
 * A Linear opMode class that is used to test different things in a TeleOp setting.
 *
 *
 * @author FTC Team #10273, The Cat in the Hat Comes Back
 */
@Disabled
@TeleOp(name="Motor Test TeleOp", group="CatTest TeleOp")
public class Test_MotorTeleOp extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime elapsedGameTime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();

    /* Declare OpMode members. */
    CatHW_Async robot = new CatHW_Async();  // Use our mecanum asynchronous hardware

    /* Constructor */
    public Test_MotorTeleOp() {}


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware
        robot.init(hardwareMap, this, false, false);

        // Define and Initialize Extra Motors: //

        // Set Extra Motor and Servo Modes: //

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Go! (Presses PLAY)
        elapsedGameTime.reset();
        elapsedTime.reset();

        // Run infinitely until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            //--------------------------------------------------------------------------------------
            // Telemetry Data:
            //--------------------------------------------------------------------------------------

            //telemetry.addData("Test Power:", "%.2f", testMotor.getPower());
            //telemetry.addData("Test Encoder:", "%d", testMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
