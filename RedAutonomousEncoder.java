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

@TeleOp(name="RedAutonomousEncoder")
public class RedAutonomousEncoder extends LinearOpMode {
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
    static final int DISTANCE_1 = 15;//inches
    static final int DISTANCE_2 = 20;//inches
    static final int DISTANCE_3 = 8;//inches
    static final int DISTANCE_4 = 10;//inches


    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS_1 = DISTANCE_1 / CIRCUMFERENCE ;
    final static int COUNTS_1 = (int)(ENCODER_CPR * ROTATIONS_1 * GEAR_RATIO);


    final static double ROTATIONS_2 = DISTANCE_2 / CIRCUMFERENCE ;
    final static int COUNTS_2 = (int)(ENCODER_CPR * ROTATIONS_2 * GEAR_RATIO);

    final static double ROTATIONS_3 = DISTANCE_3 / CIRCUMFERENCE ;
    final static int COUNTS_3 = (int)(ENCODER_CPR * ROTATIONS_3 * GEAR_RATIO);

    final static double ROTATIONS_4 = DISTANCE_1 / CIRCUMFERENCE ;
    final static int COUNTS_4 = (int)(ENCODER_CPR * ROTATIONS_1 * GEAR_RATIO);

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

        //Initilization Above
        //Autonomous Below


        moveForward(COUNTS_1);
        arm3.setPosition(1);
        moveRight(COUNTS_2);
        moveForwardWithODSCheck(COUNTS_1);
        moveRight_1(COUNTS_3);
        BeaconPusher();
        moveLeft(COUNTS_3);
        moveForward(COUNTS_1);
        moveForwardWithODSCheck(COUNTS_1 + COUNTS_2);
        moveRight_1(COUNTS_3);
        BeaconPusher();
        moveLeft(COUNTS_3);
        moveBackward(COUNTS_4);
        moveLeft(COUNTS_3);
        shoot(1000);
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


    public void moveRight(int COUNTS){
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()){

        }

    }
    public void moveRight_1(int COUNTS){
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()){
            if(ts.isPressed()){
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                mototrbackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorbackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setPower(0);

            }

        }

    }
    public void moveLeft(int COUNTS){
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() - (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() + (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() + (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() - (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()){

        }
    }

    public void moveForward(int COUNTS){
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() + (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() + (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));
        //Makes the robot go backward with shooter as the front of robot

        runToPosition();
        setPower(.5);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()){

        }

    }
    public void moveBackward(int COUNTS){
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() - (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() - (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() - (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() - (int) COUNTS));
        //Makes the robot go forward with shooter as the front of robot

        runToPosition();
        setPower(.5);
        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()){

        }

    }

    public void followTheLine(){
        while(!ts.isPressed()){
              if (ods2.getRawLightDetected() >= 2.2) {
                rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + 1000);
                leftMotor.setTargetPosition((leftMotor.getCurrentPosition()  + 1000);
              }else{
                mototrbackRight.setTargetPosition(mototrbackRight.getCurrentPosition() + 1000);
                motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition()  + 1000);
              }
            runToPosition();
            setPower(0.25);
            while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {
                telemetry.addData("Raw ODS Light ", ods2.getRawLightDetected());
                telemetry.addData("ODS Light ", ods2.getLightDetected());
                telemetry.update();
            }
        }
    }
    public void moveForwardWithODSCheck(int COUNTS){
        rightMotor.setTargetPosition((rightMotor.getCurrentPosition() + (int) COUNTS));
        leftMotor.setTargetPosition((leftMotor.getCurrentPosition() + (int) COUNTS));
        mototrbackRight.setTargetPosition((mototrbackRight.getCurrentPosition() + (int) COUNTS));
        motorbackLeft.setTargetPosition((motorbackLeft.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(0.1);


        while (opModeIsActive() && motorbackLeft.isBusy() && mototrbackRight.isBusy() &&
                rightMotor.isBusy() && leftMotor.isBusy()) {
            telemetry.addData("Raw ODS Light ", ods2.getRawLightDetected());
            telemetry.addData("ODS Light ", ods2.getLightDetected());
            telemetry.update();

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
    public void setPower(double power){
        motorbackLeft.setPower(power);
        mototrbackRight.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setPower(power);

    }

}
