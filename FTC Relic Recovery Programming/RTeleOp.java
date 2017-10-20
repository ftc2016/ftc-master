package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Driving Period")
public class RTeleOp extends OpMode {

    DcMotor motor_right;
    DcMotor motor_left;
    DcMotor motor_backleft;
    DcMotor motor_backright;
    ColorSensor sensor_color;
    Servo leftarm;
    Servo rightarm;
    Servo mainwheel;
    Servo servo_leftarm;
    Servo servo_rightarm;
    Servo servo_middleservo;
    double currentpos;

    boolean lock = true;
    boolean booleanpuns = true;
    //boolean deeznuts = gottem;
    boolean alwaysFalse = true;
    boolean combustible_lemon_is_trash = true;
    @Override
    public void init() {

        servo_leftarm = hardwareMap.servo.get("servo_armleft");
        // whitespace
        servo_rightarm = hardwareMap.servo.get("servo_armright");
        // whitespace
        servo_middleservo = hardwareMap.servo.get("servo_middleservo");
        // whitespace
        sensor_color = hardwareMap.colorSensor.get("sensor_color");
        // whitespace
        motor_left = hardwareMap.dcMotor.get("motor_left");
        // whitespace
        motor_backleft = hardwareMap.dcMotor.get("motor_backleft");
        // whitespace
        motor_backright = hardwareMap.dcMotor.get("motor_backright");
        // whitespace
        motor_right = hardwareMap.dcMotor.get("motor_right");

        sensor_color.enableLed(true);

    }


    @Override
    public void loop() {
        cs();
        telemetry.update();
        cs();
        telemetry.update();
        float x1 = gamepad1.right_stick_x, y1 = -gamepad1.right_stick_y;
        float x2 = gamepad1.left_stick_x;


        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;
        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;
        rightFrontPower += y1;
        rightBackPower += y1;

        // Handle strafing movement
        leftFrontPower += x1;
        leftBackPower -= x1;
        rightFrontPower -= x1;
        rightBackPower += x1;

        // Handle turning movement
        leftFrontPower += x2;
        leftBackPower += x2;
        rightFrontPower -= x2;
        rightBackPower -= x2;

        // Scale movement
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        if (max > 1) {
            leftFrontPower = (float)Range.scale(leftFrontPower, -max, max, -1, 1);
            leftBackPower = (float)Range.scale(leftBackPower, -max, max, -1, 1);
            rightFrontPower = (float)Range.scale(rightFrontPower, -max, max, -1, 1);
            rightBackPower = (float)Range.scale(rightBackPower, -max, max, -1, 1);
        }

        motor_backleft.setPower(-leftBackPower);
        motor_left.setPower(-leftFrontPower);
        motor_right.setPower(-rightFrontPower);
        motor_backright.setPower(-rightBackPower);

        if(currentpos == 0.01) {
            //1stpos

        }
        else if (currentpos == 0.2) {
            //2ndpos
        }
        else if(currentpos == 0.5) {
            //3rdpos
        }
        telemetry.update();

        // start of controller movements
        if(gamepad1.a) {
            mainwheel.setPosition(0.1);
            double currentpos = 0.1;
        }
        if(gamepad1.b) {
            mainwheel.setPosition(.3);
            double currentpos = .3;
        }
        if(gamepad1.x) {
            mainwheel.setPosition(.2);
            double currentpos = .2;
        }
        if(gamepad1.y) {
            mainwheel.setPosition(.5);
            double currentpos = .5;
        }
        if(gamepad2.a) {
            rightarm.setPosition(1);
            leftarm.setPosition(0);
            telemetry.addData("right arm position", rightarm.getPosition());
        }
        if(gamepad2.b){
            rightarm.setPosition(0);
            leftarm.setPosition(1);
            telemetry.addData("right arm position", rightarm.getPosition());
        }


        // end of loop
    }



    @Override
    public void stop() {
    }
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.04, 0.08, 0.9, 0.11, 0.14, 0.17, 0.23, 0.29, 0.35, 0.42, 0.49, 0.59, 0.71, 0.84, 0.99, 1.00};
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
    public void cs() {

        if (sensor_color.blue() > sensor_color.red()) {
            telemetry.addData("blue is being shown", sensor_color.blue());

        } else {
            telemetry.addData("red is being shown", sensor_color.red());
        }
    }
}
       /* public void fstpos() {

        }
    public void sndpos() {

    }
    public void trdpos() {

    }
     }
*/