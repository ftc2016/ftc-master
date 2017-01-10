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
    DcMotor motorRight;
    DcMotor motorLeft;
    //DcMotor BallLeft;
    //DcMotor BallRight;
    OpticalDistanceSensor  ods1;
    OpticalDistanceSensor  ods2;
    ColorSensor colorSensor;
    DcMotor con;
    Servo leftArm;
    Servo rightArm;

    static final int ENCODER_CPR = 1120;
    static final double GEAR_RATIO = 2;
    static final int WHEEL_DIAMETER = 4;
    static final int DISTANCE = 65;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE ;
    final static double COUNTS = ENCODER_CPR * ROTATIONS * GEAR_RATIO;



    @Override
    public void runOpMode() throws NullPointerException {
        motorLeft = hardwareMap.dcMotor.get("motor_right");

        motorRight = hardwareMap.dcMotor.get("motor_left");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //BallLeft = hardwareMap.dcMotor.get("hit_left");
        //BallRight = hardwareMap.dcMotor.get("hit_right");

        leftArm = hardwareMap.servo.get("arm1");
        rightArm = hardwareMap.servo.get("arm2");

        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        colorSensor.enableLed(false);


        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setPower(0);
        motorRight.setPower(0);
        waitForStart();

        motorRight.setTargetPosition(-(int)COUNTS);
        motorLeft.setTargetPosition(-(int)COUNTS);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(.5);
        motorRight.setPower(.5);

        while (opModeIsActive() && motorLeft.isBusy() && motorRight.isBusy()){
            telemetry.addData("left current position", motorLeft.getCurrentPosition());
            telemetry.addData("right current position", motorRight.getCurrentPosition());
            telemetry.update();

        }

        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorLeft.setTargetPosition((motorRight.getCurrentPosition() + 2000));
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(.5);

        while (opModeIsActive() && motorRight.isBusy()){
            telemetry.addData("left current position", motorLeft.getCurrentPosition());
            telemetry.addData("right current position", motorRight.getCurrentPosition());
            telemetry.update();

        }

/*        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorLeft.setTargetPosition(motorRight.getCurrentPosition() + 2000);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setTargetPosition(motorRight.getCurrentPosition() + 2000);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(.5);
        motorRight.setPower(.5);

        while (opModeIsActive() && motorRight.isBusy()){
            telemetry.addData("left current position", motorLeft.getCurrentPosition());
            telemetry.addData("right current position", motorRight.getCurrentPosition());
            telemetry.update();
            if (ods1.getRawLightDetected()  >= 1) {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }
        }*/
    }
}
