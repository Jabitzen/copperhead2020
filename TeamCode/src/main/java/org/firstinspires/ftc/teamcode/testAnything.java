package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;


@TeleOp(name="Test anything", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testAnything extends LinearOpMode{

    //RobotHw robot = new RobotHw();
    BitMapVision bm1 = null;
    String skyStonePos = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //robot.init(this);
        bm1 = new BitMapVision(this);
        skyStonePos = bm1.findRedSkystones();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("pos :", (skyStonePos));
            telemetry.update();
        }
    }
}
