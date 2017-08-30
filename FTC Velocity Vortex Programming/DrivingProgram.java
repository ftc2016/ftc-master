package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Driving")
public class DrivingProgram extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor Sweeper;
    OpticalDistanceSensor opticalDistanceSensor;
    ColorSensor colorSensor;
    TouchSensor touchSensor;
    DcMotor Hitter;

    public DrivingProgram() {

    }
    @Override
    public void init() {
        Sweeper = hardwareMap.dcMotor.get("sweeper");
        motorRight = hardwareMap.dcMotor.get("motor_right");
        motorLeft = hardwareMap.dcMotor.get("motor_left");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        //opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("optical");
        //colorSensor = hardwareMap.colorSensor.get("color");
        //touchSensor = hardwareMap.touchSensor.get("touch");
        Hitter = hardwareMap.dcMotor.get("hitter");


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
        motorLeft.setPower(left);
        if (gamepad2.a) {
            Sweeper.setPower(1.00);
        }
        if (gamepad2.b) {
            Sweeper.setPower(0);
        }
        if (gamepad2.x) {
            Sweeper.setPower(-1.00);
        }
        if (gamepad2.dpad_right) {
            Hitter.setPower(2.00);
        }
        if (gamepad2.dpad_up) {
            Hitter.setPower(0);
        }
        if (gamepad2.dpad_left) {
            Hitter.setPower(-2.00);
        }
    }

    @Override
    public void stop() {

    }


    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.04, 0.08, 0.9, 0.11, 0.14, 0.17, 0.23,
                0.29, 0.35, 0.42, 0.49, 0.59, 0.71, 0.84, 0.99, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
