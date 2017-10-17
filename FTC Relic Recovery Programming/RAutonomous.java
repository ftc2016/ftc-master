package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by ftcte on 9/29/2017.
 */

public class RAutonomous extends LinearOpMode {
    // frame count I use this to figure out how slow we need to go
    int frameCount = 0;
    public int lcr;
    // use this to refresh all assets
    OpenGLMatrix lastLocation = null;
    // I use this to initialize vuforia
    VuforiaLocalizer vuforia;
    ColorSensor colorSensor;
    DcMotor motor_left;
    DcMotor motor_right;
    DcMotor motor_backleft;
    DcMotor motor_backright;
    Servo servo_leftarm;
    Servo servo_rightarm;
    Servo servo_middleservo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo_leftarm = hardwareMap.servo.get("servo_armleft");
        // whitespace
        servo_rightarm = hardwareMap.servo.get("servo_armright");
        // whitespace
        servo_middleservo = hardwareMap.servo.get("servo_middleservo");
        // whitespace
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        // whitespace
        motor_left = hardwareMap.dcMotor.get("motor_left");
        // whitespace
        motor_backleft = hardwareMap.dcMotor.get("motor_backleft");
        // whitespace
        motor_backright = hardwareMap.dcMotor.get("motor_backright");
        // whitespace
        motor_right = hardwareMap.dcMotor.get("motor_right");

        motor_left.setDirection(DcMotor.Direction.REVERSE);
        motor_backleft.setDirection(DcMotor.Direction.REVERSE);

        /// above is defining motors and giving them the naming convention

        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Left motor current encoder position", motor_left.getCurrentPosition());
        telemetry.addData("Right motor current encoder position", motor_right.getCurrentPosition());
        telemetry.addData("backLeft motor current encoder position", motor_backleft.getCurrentPosition());
        telemetry.addData("backRight motor current encoder position", motor_backright.getCurrentPosition());


        //above is encoder setup

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // our license key is below replace it if needed
        parameters.vuforiaLicenseKey = "Ac0dfNr/////AAAAGVIj/WGQ20ausA1vrtvVr/MVsIByyuYLEuY0rlXewrVHaCzLe1iRW9q6+nvnKQOcZk7Sg2eOfib/cpA7NbtqD7E6tD2FegRNKdqTLlVwNE4oT2/Sv60VBMyMAEUSMk8ZTXMZ/4alBqwUqFe2ajodtauM+Vf2SGL1/GPcaeCvEDwK0J7mr2ggfyLcLKFcky3oZCrYOlRGKGKLbOkAFOUbJrMxrbjbcKCrP9vH4F3Sf2ArJIJnij+WzVk7NcLe25Sml0rppRjHvMscSjfHvK1U36G02f6SimOWPBu3zekvAuqJ+kG5Tl3WvlsLZLGzv8R35ovQYra9cYrZzhf7CdmGEo6HhDXaQdt3mWzWby7L30Nn";
        // if you do replace it please upload new .svg files to the developer.vuforia.com
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        colorSensor.enableLed(true);
        //at this point everything is readu to go, the above happens when you press init and as soon as you press play everything starts
        waitForStart();
        //int TETRIX_TICKS_PER_REV = 1440;
        // I think where using andymark
        int ANDYMARK_TICKS_PER_REV = 1120;
        //activate vuforia

  // autonomous
        //relicTrackables.activate();
        //cypher(relicTemplate);
        //relicTrackables.deactivate();



// autonomous

    }

    public String cypher(VuforiaTrackable relicTemplate) {

        //checks everything
        RelicRecoveryVuMark mark = RelicRecoveryVuMark.UNKNOWN;
        int leftCount = 0;
        int centerCount = 0;
        int rightCount = 0;

        for (int i = 0; i < 20; i++) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            sleep(100);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    leftCount++;

                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    centerCount++;

                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    rightCount++;

                }
            }
        }
        // more variable checking
        if (leftCount > centerCount && leftCount > rightCount) {
            mark = RelicRecoveryVuMark.LEFT;
            lcr = 1;
        } else if (centerCount > leftCount && centerCount > rightCount) {
            mark = RelicRecoveryVuMark.CENTER;
            lcr = 2;
        } else if (rightCount > centerCount && rightCount > leftCount) {
            mark = RelicRecoveryVuMark.RIGHT;
            lcr = 3;
        }
        // prints key for cryptobox
        telemetry.update();
        telemetry.addData("Key: ", mark.toString());
        telemetry.update();
        return "abc";
    }
    public void lcrkey() {
        telemetry.addData("1 is Left, 2 is center, 3 is right value is ",lcr);
    }
    public void ColorSensor() {
       if(colorSensor.blue() > colorSensor.red()) {
           telemetry.addData("blue is being shown", colorSensor.blue());

       }else{
           telemetry.addData("red is being shown", colorSensor.red());
       }


    }
    public void DriveForwardDistance(double power, int distance)
    {
        motor_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_backright.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_backleft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_right.setMode(DcMotor.RunMode.RESET_ENCODERS);

        motor_left.setTargetPosition(distance);
        motor_backleft.setTargetPosition(distance);
        motor_right.setTargetPosition(distance);
        motor_backright.setTargetPosition(distance);

        motor_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PowerSetter(power);

        while(motor_left.isBusy() && motor_backleft.isBusy() && motor_right.isBusy() && motor_backright.isBusy()) {
            //this is like a sleep
        }
        StopDriving();
        motor_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
       public void PowerSetter (double power) {
        motor_backright.setPower(power);
        motor_left.setPower(power);
        motor_right.setPower(power);
        motor_backleft.setPower(power);


    }

    public void StopDriving(){
        PowerSetter(0);
    }
    public void turnpowers(double powerleft, double powerright) {
        motor_backleft.setPower(powerleft);
        motor_left.setPower(powerleft);
        // not confirmed to work
        motor_right.setPower(powerright);
        motor_backright.setPower(powerright);

    }
    public void slideleft(double power) {
        motor_left.setPower(-power);
        motor_backleft.setPower(power);
        motor_right.setPower(power);
        motor_backright.setPower(-power);

    }
    public void slideright(double power) {
        motor_left.setPower(power);
        motor_backleft.setPower(-power);
        motor_right.setPower(-power);
        motor_right.setPower(power);
    }
    public void colorSensorRed() {
        colorSensor.enableLed(true);
        if(colorSensor.red() >= 15 ) {
            PowerSetter(0.2);
            sleep(1000);
        }else{
            PowerSetter(-0.2);
            sleep(1000);
        }

    }
    public void colorSensorBlue() {
        colorSensor.enableLed(true);
        if (colorSensor.blue() >= 15) {
            PowerSetter(0.2);
            sleep(1000);
        } else {
            PowerSetter(-0.2);
            sleep(1000);
        }
    }
        public void servoArm() {

    }
    }
}

