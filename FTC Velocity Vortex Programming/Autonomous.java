package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Vishnu2004 on 10/22/2016.
 */
@TeleOp(name="AutonomousPreRegional")
public class Autonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor Sweeper;
//    OpticalDistanceSensor opticalDistanceSensor;
 //   ColorSensor colorSensor;
  //  TouchSensor touchSensor;
    DcMotor Hitter;


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor  = hardwareMap.dcMotor.get("motor_left");
        rightMotor = hardwareMap.dcMotor.get("motor_right");
        Hitter = hardwareMap.dcMotor.get("hitter");
        //opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("optical");
        //colorSensor = hardwareMap.colorSensor.get("color");
        //touchSensor = hardwareMap.touchSensor.get("touch");
       // servoC = hardwareMap.servo.get("servo");
        Sweeper = hardwareMap.dcMotor.get("sweeper");
       // float hsvValues[] = {0F,0F,0F};
     //   final float values[] = hsvValues;
   //     final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
       // boolean bPrevState = false;
       // boolean bCurrState = false;

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        //move forward to shoot
        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(300);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        Hitter.setPower(.7);
        sleep(400);
        Hitter.setPower(0);

        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(1900);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(500);

 /*       leftMotor.setPower(-.5);
        rightMotor.setPower(-.5);
        sleep(700);

        leftMotor.setPower(0);
        rightMotor.setPower(-.5);
        sleep(1000);

        leftMotor.setPower(-.5);
        rightMotor.setPower(-.5);
        sleep(1000);*/
        /*        Sweeper.setPower(-1.00);
        sleep(5000);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(500);

/*        boolean bLedOn;
        bLedOn = false;
        colorSensor.enableLed(bLedOn);

        while (opModeIsActive()) {
            bCurrState = gamepad1.x;
            bPrevState = bCurrState;
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            if(colorSensor.red() > 5){
                telemetry.addData("Red Detected", colorSensor.red());
                //servoC.setPosition();
            }


            if(colorSensor.blue() > 5) {
                telemetry.addData("Blue Detected", colorSensor.blue());
            }


            telemetry.addData("Blue Color Values", colorSensor.blue());
            telemetry.addData("Red Color Values", colorSensor.red());
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();
        }
*/

/*        double reflectance = opticalDistanceSensor.getLightDetected();
        double raw = opticalDistanceSensor.getRawLightDetected();
//        long expire = System.currentTimeMillis() + 5200;

        while (!touchSensor.isPressed()){
            //       while (System.currentTimeMillis() < expire){
            if (reflectance >= 0.25) {
                rightMotor.setPower(-.2);
                leftMotor.setPower(0);
            }
            //Otherwise (if the sensor is off the line)
            //Only the left motor rotates to move it back toward the line
            else {
                leftMotor.setPower(-0.2);
                rightMotor.setPower(0);
            }

        }*/

        //telemetry.addData("Left Target Position",leftMotor.getCurrentPosition());
        //telemetry.addData("Right Target Position",rightMotor.getCurrentPosition());
        //telemetry.addData("Touch Sensor ", touchSensor.isPressed());
//        telemetry.addData("Raw Reflectance", raw);
 //       telemetry.addData("Reflectance", reflectance);

    }
}
