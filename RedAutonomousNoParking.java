package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Red Autonomous No Parking")
public class RedAutonomousNoParking extends LinearOpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor motorbackLeft;
    DcMotor mototrbackRight;
    DcMotor Ball;
    Servo arm1;
    Servo arm2;
    TouchSensor ts;
    Servo arm3;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;


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


        waitForStart();
        moveForward(3500, 0.1);

        arm3.setPosition(1);

        moveRight(2500, 0.3);

        moveforwardWithODSCheck(0.075);

        moverightuntilPressed(0.25);

        BeaconPusher();

        moveLeft(480, 0.4);

        turnLeft(200);

        moveForward(600, 0.1);

        moveforwardWithODSCheck(0.075);

        moverightuntilPressed(0.25);

        BeaconPusher();

        turn(1200);

        moveBackward(500, 0.6);

        shoot(400);
        stop(500);
        //End of autonomous

    }

    private void updateTelemetryStatus() {
        telemetry.addData("ODS Raw", ods2.getRawLightDetected());
        telemetry.addData("ODS ", ods2.getLightDetected());
        telemetry.addData("ODS Raw 1 ", ods1.getRawLightDetected());
        telemetry.addData("Color sensor red", colorSensor.red());
        telemetry.addData("Color sensor blue", colorSensor.blue());
        telemetry.update();
        sleep(1000);
    }

    public void BeaconPusher() {

        telemetry.addData("Blue Color", colorSensor.blue());
        telemetry.addData("Red Color", colorSensor.red());
        telemetry.update();
        sleep(500);
        if (colorSensor.red() > 10) {
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

    public void moveForward(int i, double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        mototrbackRight.setPower(power);
        motorbackLeft.setPower(power);
        sleep(i);
        stop(100);
    }

    public void moveBackward(int i, double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
        mototrbackRight.setPower(-power);
        motorbackLeft.setPower(-power);
        sleep(i);
        stop(100);
    }
    public void moveforwardWithODSCheck(double power) {
        int i = 0;
        while (i < 10000 ) {
            if (ods2.getRawLightDetected() >= 2.2 || ods1.getRawLightDetected() >= 2.2) {
                break;
            } else {
                leftMotor.setPower(power);
                rightMotor.setPower(power);
                mototrbackRight.setPower(power);
                motorbackLeft.setPower(power);
                sleep(1);
            }
            i++;
        }
        stop(1000);
        telemetry.addData("Raw ODS Light ", ods2.getRawLightDetected());
        telemetry.addData("ODS Light ", ods2.getLightDetected());
        telemetry.update();
        sleep(2000);
    }
    public void stop(int time){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        mototrbackRight.setPower(0);
        motorbackLeft.setPower(0);
        sleep(time);

    }

    public void moveRight(int i, double power) {
        rightMotor.setPower(power);
        mototrbackRight.setPower(-power);
        leftMotor.setPower(-power);
        motorbackLeft.setPower(power);
        sleep(i);
        stop(100);
    }

    public void moveLeft(int i, double power) {
        rightMotor.setPower(-power);
        mototrbackRight.setPower(power);
        leftMotor.setPower(power);
        motorbackLeft.setPower(-power);
        sleep(i);
        stop(100);
    }

    public void turnLeft(int i) {
        rightMotor.setPower(0.3);
        mototrbackRight.setPower(0.3);
        leftMotor.setPower(0);
        motorbackLeft.setPower(0);
        sleep(i);
    }

    public void shoot(int i) {
        Ball.setPower(1);
        sleep(i);
        Ball.setPower(0);
        sleep(100);

    }
    public void turn(long i){
        rightMotor.setPower(-0.3);
        mototrbackRight.setPower(-0.3);
        leftMotor.setPower(0);
        motorbackLeft.setPower(0);
        sleep(i);

    }

    public void moverightuntilPressed(double power) {
        boolean tspressed = false;
        while (!tspressed && opModeIsActive()) {
            rightMotor.setPower(power);
            mototrbackRight.setPower(-power);
            leftMotor.setPower(-power);
            motorbackLeft.setPower(power);
            if (ts.isPressed()) {
                tspressed = true;
            }
        }
        stop(100);
    }



    }


