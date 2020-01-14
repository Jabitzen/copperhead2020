package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;


@Autonomous(name="Test anything", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testAnything extends LinearOpMode{

    RobotHw robot = new RobotHw();
    BitMapVision bm1 = null;
    ElapsedTime runTime = new ElapsedTime();

    //String skyStonePos = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        waitForStart();
        robot.approachStonesRed(.4);
        sleep(5000);
        robot.strafeLeftGyro(80, .6, 5);
        sleep(30000);
        double fRtickspersecond  = 0.0;
        double fLtickspersecond  = 0.0;
        double bLtickspersecond  = 0.0;
        double bRtickspersecond  = 0.0;
/*
        robot.init(this);
        waitForStart();
*/

        // Full power test

        //robot.goStraightGyro(40, .5, 6);
        //sleep(1000000);


        while(robot.fR.getCurrentPosition() < 10000)
        {
            robot.fR.setPower(0.9);
            robot.fL.setPower(1000.0/1037.0);
            robot.bL.setPower(1);
            robot.bR.setPower(0.5);

            telemetry.addData("fL:", robot.fL.getCurrentPosition());
            telemetry.addData("fR:", robot.fR.getCurrentPosition());
            telemetry.addData("bL:", robot.bR.getCurrentPosition());
            telemetry.addData("bR:", robot.bL.getCurrentPosition());
            telemetry.update();

        }

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.fR.setPower(0);
        robot.fL.setPower(0);
        robot.bL.setPower(0);
        robot.bR.setPower(0);

        sleep(1000000);

        // Ticks per second tests
        sleep(30000);
        runTime.reset();
        while(runTime.seconds() < 10)
        {
            robot.fR.setPower(1);
            robot.fL.setPower(1);
            robot.bL.setPower(1);
            robot.bR.setPower(1);


        }
        robot.fR.setPower(0);
        robot.fL.setPower(0);
        robot.bL.setPower(0);
        robot.bR.setPower(0);

        fRtickspersecond = robot.fR.getCurrentPosition()/10;
        fLtickspersecond = robot.fL.getCurrentPosition()/10;
        bRtickspersecond = robot.bR.getCurrentPosition()/10;
        bLtickspersecond = robot.bL.getCurrentPosition()/10;

        telemetry.addData("fRtickspersecond", fRtickspersecond);
        telemetry.addData("fLtickspersecond", fLtickspersecond);
        telemetry.addData("bRtickspersecond", bRtickspersecond);
        telemetry.addData("bLtickspersecond", bLtickspersecond);
        telemetry.update();

        // Grabber tests
        sleep(300000);

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
