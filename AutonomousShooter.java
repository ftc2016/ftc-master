
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="AutonomousShooter")
public class AutonomousShooter extends LinearOpMode {
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
        Ball1 = hardwareMap.dcMotor.get("shoot1");
        Ball2 = hardwareMap.dcMotor.get("shoot2");
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

        moveForward(2660, 0.1);

        shoot(500);

        moveForward(2800, 0.1);

        stop(1);

    }


    public void moveForward(int i, double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
        mototrbackRight.setPower(-power);
        motorbackLeft.setPower(-power);
        sleep(i);
        stop(100);
    }

    public void stop(int time){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        mototrbackRight.setPower(0);
        motorbackLeft.setPower(0);
        sleep(time);

    }


    public void shoot(int i) {
        Ball1.setPower(1);
        sleep(i);
        Ball1.setPower(0);
        sleep(100);
        Ball2.setPower(1);
        sleep(i);
        Ball2.setPower(0);
        sleep(100);

    }

}

