package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;


@Autonomous(name="Test anything", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testAnything extends LinearOpMode{

    RobotHw robot = new RobotHw();
    BitMapVision bm1 = null;
    //String skyStonePos = null;

    @Override
    public void runOpMode() throws InterruptedException {



        robot.init(this);
        waitForStart();

        robot.grabberBDown();
        sleep (1000);
        robot.grabberBUp();
        sleep(1000);
        robot.gripBUp();
        sleep(1000);
        robot.gripBDown();
        sleep(1000);

//        robot.clawDown();
//        sleep(3000);
//        robot.clawUp();
//        sleep(3000);

//        robot.grabberRDown();
//
//        sleep(2000);
//        robot.gripRUp();
//        telemetry.addLine("gripRUp");
//        telemetry.update();
//        sleep(5000);
//
//
//        robot.gripRDown();
//        telemetry.addLine("gripRDown");
//        telemetry.update();
//
//        sleep(5000);
//
//        robot.grabberRUp();
//        sleep(5000);


    }
}
