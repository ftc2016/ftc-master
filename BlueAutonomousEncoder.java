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

@TeleOp(name="BlueAutonomousEncoder")
public class BlueAutonomousEncoder extends LinearOpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor motorbackLeft;
    DcMotor mototrbackRight;
    DcMotor con;
    DcMotor Ball1;
    DcMotor Ball2;
    Servo arm1;
    Servo arm2;
    TouchSensor ts;
    ColorSensor colorSensor;
    Servo arm3;
    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;
    //DcMotor lift;

    static final int ENCODER_CPR = 1120;
    static final double GEAR_RATIO = 2;
    static final int WHEEL_DIAMETER = 4;
    static final int DISTANCE_1 = 15;//inches
    static final int DISTANCE_2 = 22;//inches
    static final int DISTANCE_3 = 8;


    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS_1 = DISTANCE_1 / CIRCUMFERENCE;
    final static int COUNTS_1 = (int) (ENCODER_CPR * ROTATIONS_1 * GEAR_RATIO);


    final static double ROTATIONS_2 = DISTANCE_2 / CIRCUMFERENCE;
    final static int COUNTS_2 = (int) (ENCODER_CPR * ROTATIONS_2 * GEAR_RATIO);

    final static double ROTATIONS_3 = DISTANCE_3 / CIRCUMFERENCE;
    final static int COUNTS_3 = (int) (ENCODER_CPR * ROTATIONS_3 * GEAR_RATIO);

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
        Ball1 = hardwareMap.dcMotor.get("shoot1");
        Ball2 = hardwareMap.dcMotor.get("shoot2");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");

        //lift = hardwareMap.dcMotor.get("lift");

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

        //Initilization Above
        //Autonomous Below

        moveForward(COUNTS_1 - 1000);
        shoot(400);
        moveForward(900);
        moveRight(COUNTS_2 + 900);
        moveForwardWithODSCheck(COUNTS_1);
        sleep(250);
        moveRight_1(COUNTS_3);
        BeaconPusher();
        moveLeft(COUNTS_3, .35);
        moveForward(COUNTS_1);
        moveForwardWithODSCheck(COUNTS_1 + COUNTS_2);
        sleep(250);
        moveRight_1(COUNTS_3);
        BeaconPusher();
        moveLeft(COUNTS_3, .35);
        moveBackward(COUNTS_1);
        moveLeft(COUNTS_1 + COUNTS_1 + 250, .6);
    }


    //Method Details Below


    public void BeaconPusher() {

        telemetry.addData("Blue Color", colorSensor.blue());
        telemetry.addData("Red Color", colorSensor.red());
        telemetry.update();
        sleep(100);
        if (colorSensor.blue() > 10) {
            arm2.setPosition(1);
            sleep(1000);
            arm2.setPosition(0);
            sleep(1000);
        } else {
            arm1.setPosition(1);
            sleep(1000);
            arm1.setPosition(0);
            sleep(1000);
        }

    }


    public void moveRight(int COUNTS) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {

        }

    }

    public void moveRight_1(int COUNTS) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {
            if (ts.isPressed()) {
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mototrbackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorbackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPower(0);

            }

        }

    }

    public void moveLeft(int COUNTS, double power) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() - (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() + (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() + (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() - (int) COUNTS));

        runToPosition();
        setPower(power);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {

        }
    }

    public void moveForward(int COUNTS) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() - (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() - (int) COUNTS));

        runToPosition();
        setPower(.5);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {

        }

    }
    public void moveBackward(int COUNTS) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() + (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() + (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.5);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {

        }

    }

    public void moveBackwardWithODSCheck(int COUNTS) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() + (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() + (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(0.1);


        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {
            if (ods2.getRawLightDetected() >= 2.2) {
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mototrbackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorbackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPower(0);
            }
        }

    }


    public void moveForwardWithODSCheck(int COUNTS) {
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() - (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() - (int) COUNTS));

        runToPosition();
        setPower(0.1);


        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {
            if (ods2.getRawLightDetected() >= 2.2) {
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mototrbackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorbackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPower(0);
            }
        }

    }

    public void shoot(int i) {
        Ball1.setPower(1);
        sleep(i);
        Ball1.setPower(0);
        sleep(100);
        Ball2.setPower(-1);
        sleep(i);
        Ball2.setPower(0);
        sleep(100);
    }

    public void runToPosition() {
        motorbackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mototrbackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setPower(double power) {
        motorbackLeft.setPower(power);
        mototrbackRight.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setPower(power);

    }

}



