package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Autonomous Encoder")
public class AutonomousEncoder extends LinearOpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor motorbackLeft;
    DcMotor mototrbackRight;
    DcMotor con;
    DcMotor Ball;
    Servo arm1;
    Servo arm2;
    TouchSensor ts;
    Servo arm3;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;

    static final int ENCODER_CPR = 1120;
    static final double GEAR_RATIO = 2;
    static final int WHEEL_DIAMETER = 4;
    static final int DISTANCE = 65;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE ;
    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;



    @Override
    public void runOpMode() throws InterruptedException {
        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        rightMotor = hardwareMap.dcMotor.get("motor_right");
        leftMotor = hardwareMap.dcMotor.get("motor_left");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft = hardwareMap.dcMotor.get("motor_backleft");
        motorbackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mototrbackRight = hardwareMap.dcMotor.get("motor_backright");
        Ball = hardwareMap.dcMotor.get("shoot");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");

        ts = hardwareMap.touchSensor.get("ts");
        arm3 = hardwareMap.servo.get("arm3");
        colorSensor.enableLed(false);

        arm1.setDirection(Servo.Direction.REVERSE);
        arm2.setDirection(Servo.Direction.REVERSE);

        arm1.setPosition(0);
        arm2.setPosition(0);

        rightMotor.setPower(0);
        leftMotor.setPower(0);
        motorbackLeft.setPower(0);
        mototrbackRight.setPower(0);

        waitForStart();

        motorbackLeft.setTargetPosition(-(int)COUNTS);
        mototrbackRight.setTargetPosition(-(int)COUNTS);
        rightMotor.setTargetPosition(-(int)COUNTS);
        leftMotor.setTargetPosition(-(int)COUNTS);

        runToPosition();

        motorbackLeft.setPower(.5);
        mototrbackRight.setPower(.5);
        rightMotor.setPower(.5);
        leftMotor.setPower(.5);

        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                    rightMotor.isBusy() && leftMotor.isBusy()){
            telemetry.addData("left current position", rightMotor.getCurrentPosition());
            telemetry.addData("right current position", leftMotor.getCurrentPosition());
            telemetry.addData("left back current position", motorbackLeft.getCurrentPosition());
            telemetry.addData("right back current position", mototrbackRight.getCurrentPosition());
            telemetry.update();

        }

        shoot(1000);
        motorbackLeft.setPower(0);
        mototrbackRight.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setPower(0);

        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + 2000));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - 2000));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - 2000));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + 2000));

        runToPosition();

        motorbackLeft.setPower(.5);
        mototrbackRight.setPower(.5);
        rightMotor.setPower(.5);
        leftMotor.setPower(.5);


        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()){
            telemetry.addData("left current position", rightMotor.getCurrentPosition());
            telemetry.addData("right current position", leftMotor.getCurrentPosition());
            telemetry.addData("left back current position", motorbackLeft.getCurrentPosition());
            telemetry.addData("right back current position", mototrbackRight.getCurrentPosition());
            telemetry.update();

        }
    }
    public void shoot(int i) {
        Ball.setPower(1);
        sleep(i);
        telemetry.addData("Shooting ", "");
        telemetry.update();
        sleep(100);
        Ball.setPower(0);
        sleep(100);

    }
    public void runToPosition(){
        motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mototrbackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

}
