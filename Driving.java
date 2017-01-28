package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Driving Period")
public class Driving extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorbackLeft;
    DcMotor mototrbackRight;
    DcMotor con;
    DcMotor Ball;
    Servo arm1;
    Servo arm2;

    @Override
    public void init() {
        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft = hardwareMap.dcMotor.get("motor_backleft");
        motorbackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mototrbackRight = hardwareMap.dcMotor.get("motor_backright");
        Ball = hardwareMap.dcMotor.get("shoot");
    }

    @Override
    public void loop() {
        float throttle = gamepad1.left_stick_y;
        float direction = -gamepad1.right_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);
        // write the values to the motors
        motorRight.setPower(right);
        mototrbackRight.setPower(right);
        motorLeft.setPower(left);
        motorbackLeft.setPower(left);
        boolean down;
        if (gamepad1.dpad_up) {
            Ball.setPower(-1);
        }
        if (gamepad2.x) {
            arm1.setPosition(1);
        }
        if (gamepad2.y) {
            arm1.setPosition(0);
            arm2.setPosition(0);
        }
        if (gamepad2.b) {
            arm2.setPosition(1);
        }
        if (gamepad2.a) {
            arm2.setPosition(1);
            arm1.setPosition(1);
        }
        float throttle1 = gamepad2.left_stick_y;
        float direction1 = -gamepad2.right_stick_x;
        float right1 = throttle1 - direction1;
        float left1 = throttle1 + direction1;
        right1 = Range.clip(right, -1, 1);
        left1 = Range.clip(left, -1, 1);
        right1 = (float) scaleInput(right);
        left1 = (float) scaleInput(left);
        // write the values to the motors
        motorRight.setPower(-right1);
        mototrbackRight.setPower(right1);
        motorLeft.setPower(-left1);
        motorbackLeft.setPower(left1);
        telemetry.addData("Right front : " , motorRight.getPower());
        telemetry.addData("Right back : " , mototrbackRight.getPower());
        telemetry.addData("Left front : " , motorLeft.getPower());
        telemetry.addData("Left back : " , motorbackLeft.getPower());
        telemetry.update();


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
}
