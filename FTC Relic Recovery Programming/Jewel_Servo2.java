package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Red Auto")
public class Jewel_Servo2 extends LinearOpMode {


    private Servo jewelVertical = null;
    private Servo jewelHorizontal = null;
    private ColorSensor jewelColor = null;

    public int lcr;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        jewelVertical = hardwareMap.get(Servo.class, "jewel_vertical");
        jewelHorizontal = hardwareMap.get(Servo.class,"jewel_horizontal");
        jewelColor = (ColorSensor) hardwareMap.get("jewel_color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery




        waitForStart();
        jewelColor.enableLed(true);

        while (opModeIsActive()) {
            //jewelColor.enableLed(true);
            jewelVertical.setDirection(Servo.Direction.FORWARD);
            jewelVertical.setPosition(.25);
            sleep(4000);

            //if (jewelColor.blue() <= 7) {
            if (jewelColor.blue() > 0) {
                telemetry.addData("Blue Color:", jewelColor.blue());
                telemetry.addData("Red Color:", jewelColor.red());
                telemetry.update();
                // jewelHorizontal.setDirection(Servo.Direction.REVERSE);
                jewelHorizontal.setPosition(-0.5);
                sleep (1000);
                // jewelHorizontal.setDirection(Servo.Direction.FORWARD);
                jewelHorizontal.setPosition(0.34);
            } else {
                telemetry.addData("Red Color:", jewelColor.red());
                telemetry.addData("Blue Color:", jewelColor.blue());
                telemetry.update();
                //  jewelHorizontal.setDirection(Servo.Direction.FORWARD);
                jewelHorizontal.setPosition(0.5);
                sleep (1000);
                // jewelHorizontal.setDirection(Servo.Direction.REVERSE);
                jewelHorizontal.setPosition(.34);
            }
            sleep (1000);
            jewelVertical.setDirection(Servo.Direction.REVERSE);
            jewelVertical.setPosition(-.25);
            sleep (1000);
            requestOpModeStop();
        }
    }
}