package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Vishnu2004 on 10/22/2016.
 */
@TeleOp(name="AutonomousRegional")
public class RegionalAutonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor Sweeper;
    DcMotor Hitter;


    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor  = hardwareMap.dcMotor.get("motor_left");
        rightMotor = hardwareMap.dcMotor.get("motor_right");
        Hitter = hardwareMap.dcMotor.get("hitter");
        Sweeper = hardwareMap.dcMotor.get("sweeper");
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(300);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        Hitter.setPower(.7);
        sleep(400);
        Hitter.setPower(0);

        leftMotor.setPower(.5);
        rightMotor.setPower(.5);
        sleep(2100);

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(750);

 }
}