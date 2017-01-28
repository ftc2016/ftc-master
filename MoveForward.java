package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Move Forward")
public class MoveForward extends LinearOpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor motorbackLeft;
    DcMotor mototrbackRight;
    DcMotor con;
    DcMotor Ball;
    Servo arm1;
    Servo arm2;
    ColorSensor colorSensor;
    OpticalDistanceSensor ods1;
    OpticalDistanceSensor  ods2;


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

        colorSensor.enableLed(false);

        arm1.setDirection(Servo.Direction.REVERSE);
        arm2.setDirection(Servo.Direction.REVERSE);
        arm1.setPosition(0);
        arm2.setPosition(0);
        rightMotor.setPower(0);
        leftMotor.setPower(0);


        waitForStart();

//        leftMotor.setPower(-0.2);
//        rightMotor.setPower(-0.2);
//        mototrbackRight.setPower(-0.2);
//        motorbackLeft.setPower(-0.2);
//        sleep(4000);

        int i = 0;
        telemetry.addData("ODS2 Light ", ods2.getRawLightDetected());
        telemetry.addData("ODS1 Light ", ods1.getRawLightDetected());
        telemetry.addData("Blue ", colorSensor.blue());
        telemetry.addData("Red ", colorSensor.red());
        telemetry.update();
        sleep(1000);
        while (i < 5000) {
            if (ods2.getRawLightDetected()  >= 2.4 || ods1.getRawLightDetected() >=2.4 ){
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                mototrbackRight.setPower(0);
                motorbackLeft.setPower(0);
                break;
            }else{
                leftMotor.setPower(-0.08);
                rightMotor.setPower(-0.08);
                mototrbackRight.setPower(-.08);
                motorbackLeft.setPower(-.08);
                sleep(1);
            }
            i++;
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        motorbackLeft.setPower(0);
        mototrbackRight.setPower(0);


    }

}

