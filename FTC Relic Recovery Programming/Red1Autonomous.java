/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Red1")
public class Red1Autonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private Servo jewelServo = null;
    private ColorSensor jewelColor = null;

    static final int ENCODER_CPR = 1120;
    static final double GEAR_RATIO = 2;
    static final int WHEEL_DIAMETER = 4;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    static final int DISTANCE_1 = 15;//inches

    final static double ROTATIONS_1 = DISTANCE_1 / CIRCUMFERENCE;
    final static int COUNTS_1 = (int) (ENCODER_CPR * ROTATIONS_1 * GEAR_RATIO);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = (DcMotor) hardwareMap.get("left_drive");
        rightDrive = (DcMotor) hardwareMap.get("right_drive");
        leftBackDrive = (DcMotor) hardwareMap.get("left_back_drive");
        rightBackDrive = (DcMotor) hardwareMap.get("right_back_drive");
        jewelServo = (Servo) hardwareMap.get("jewel_servo");
        jewelColor = (ColorSensor) hardwareMap.get("jewel_color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        jewelServo.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        jewelServo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            jewelServo.getDirection();
            telemetry.addData("Servo Direction:", jewelServo.getDirection());
            telemetry.update();
            jewelServo.setPosition(1);

            if (jewelColor.blue() == 7) {
                telemetry.addData("Blue Color:", jewelColor.blue());
                telemetry.update();
                moveForwardOdometery(.2, 500);
            } else {
                telemetry.addData("Red Color:", jewelColor.red());
                telemetry.update();
                moveBackwardOdometery(-.2, 500);
            }

            moveForward(COUNTS_1);
        }
    }
        private void moveForwardOdometery(double a, int b){
        leftDrive.setPower(a);
        leftBackDrive.setPower(a);
        rightDrive.setPower(a);
        rightBackDrive.setPower(a);
        sleep(b);
        }
        private void moveBackwardOdometery(double a, int b){
        leftDrive.setPower(a);
        leftBackDrive.setPower(a);
        rightDrive.setPower(a);
        rightBackDrive.setPower(a);
        sleep(b);
        }
    public void moveRight(int COUNTS){
        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() + (int) COUNTS));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() - (int) COUNTS));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() - (int) COUNTS));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()){

        }

    }
    public void moveRight_1(int COUNTS){
        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() + (int) COUNTS));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() - (int) COUNTS));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() - (int) COUNTS));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() + (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()){
            }

        }
    public void moveLeft(int COUNTS){
        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() - (int) COUNTS));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() + (int) COUNTS));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() + (int) COUNTS));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() - (int) COUNTS));

        runToPosition();
        setPower(.35);
        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()){

        }
    }

    public void moveForward(int COUNTS){
        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() + (int) COUNTS));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() + (int) COUNTS));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() + (int) COUNTS));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() + (int) COUNTS));
        //Makes the robot go backward with shooter as the front of robot

        runToPosition();
        setPower(.5);
        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()){

        }

    }
    public void moveBackward(int COUNTS){
        rightDrive.setTargetPosition((rightDrive.getCurrentPosition() - (int) COUNTS));
        leftDrive.setTargetPosition((leftDrive.getCurrentPosition() - (int) COUNTS));
        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition() - (int) COUNTS));
        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition() - (int) COUNTS));
        //Makes the robot go forward with shooter as the front of robot

        runToPosition();
        setPower(.5);
        while (opModeIsActive() && leftBackDrive.isBusy() && rightBackDrive.isBusy() &&
                rightDrive.isBusy() && leftDrive.isBusy()){

        }

    }
    public void runToPosition(){
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void setPower(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

    }
    }

