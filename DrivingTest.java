package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Driving Test")
public class DrivingTest extends OpMode {

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
        boolean down;
        if (gamepad1.dpad_up) {
            Ball.setPower(1);
        }
        if (gamepad1.dpad_down) {
            Ball.setPower(0);
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

 //       while (opModeIsActive()) {
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

            motorbackLeft.setPower(leftBackPower);
            motorLeft.setPower(leftFrontPower);
            motorRight.setPower(rightFrontPower);
            mototrbackRight.setPower(rightBackPower);

            // Here you set the motors' power to their respected power double.
            telemetry.addData("Right front : " , motorRight.getPower());
            telemetry.addData("Right back : " , mototrbackRight.getPower());
            telemetry.addData("Left front : " , motorLeft.getPower());
            telemetry.addData("Left back : " , motorbackLeft.getPower());
            telemetry.update();


 //       }


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
