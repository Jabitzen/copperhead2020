package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Test anything", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testAnything extends LinearOpMode{

    RobotHw robot = new RobotHw();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this);
        waitForStart();

        while (opModeIsActive()) {
            robot.fL.setPower(1);
            robot.fR.setPower(1);
            robot.bL.setPower(1);
            robot.bR.setPower(1);
            telemetry.addData("FL :", robot.fL.getCurrentPosition());
            telemetry.addData("FR :", robot.fR.getCurrentPosition());
            telemetry.addData("BL :", robot.bL.getCurrentPosition());
            telemetry.addData("BR :", robot.bR.getCurrentPosition());
            //telemetry.addData("FR :", robot.fR.getCurrentPosition());
            //telemetry.addData("BL :", robot.bL.getCurrentPosition());
            //telemetry.addData("BR :", robot.bR.getCurrentPosition());
            telemetry.update();
        }
    }
}
