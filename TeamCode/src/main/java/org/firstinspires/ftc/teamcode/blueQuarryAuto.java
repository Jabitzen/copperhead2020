package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;

@Autonomous(name="blueQuarryAuto", group="12596")
public class blueQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;
    String skyStonePos = null;
    RobotHw robot = new RobotHw();
    double correction = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        bm1 = new BitMapVision(this);
        robot.init(this);

        while (!isStarted())
        {
            skyStonePos = bm1.findBlueSkystones();
            telemetry.addData("stone", skyStonePos);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // Reset tracking angle
            robot.resetAngle();
            //Move to the 1st stone
            robot.goStraightGyro(18, .7, 3);
            // Rotate to align grabberR with stone
            robot.turnPID(90, .78/90, 0, .2, 10);
            // Align with center stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(-2, 0.2, 1);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-11, 0.2, 3);
            } else {
                robot.goStraightGyro(3.5, 0.2, 3);
            }
            //prime grabber and grip for stone
            robot.gripBUp();
            robot.grabberBDown();
            sleep(500);
            // reset encoders for backtracking
            robot.reset();
            // Approach stone
            robot.approachStonesBlue(.5);
            //grab stone
            robot.gripBDown();
            sleep(1000);
            //pick up stone
            robot.grabberBUp();
            // Pull Stone out
            robot.strafeRightGyro((robot.encoderAvg() * (11.0/537.6)) + 4.5, .6);
            //calculate angle needed to correct
            correction = robot.correctAngle(90) ;
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .13/correction, 0, 0, 1);

            // Testing telemetry
            /*
            telemetry.addData("start angle: ", startPos.firstAngle);
            telemetry.addData("angle : ", robot.getAngle());
            telemetry.addData("correct angle : ", correction);
            telemetry.update();
            */

            // Drive to deposit 1st stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(-60, 1, 7);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-66, 1, 7);
            } else {
                robot.goStraightGyro(-66, 1, 7);
            }
            robot.strafeLeftGyro(4, .5, 5);
            // grabberR lets go of stone
            robot.grabberBDown();
            robot.gripBUp();
            sleep(300);
            //put grabber up to go back to quarry
            robot.gripBDown();
            robot.grabberBUp();
            robot.strafeRightGyro(4, .5);
            //go back to quarry
            if (skyStonePos == "center") {
                robot.goStraightGyro(89.5, 1, 5);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(94.5, 1, 5);
            } else {
                robot.goStraightGyro(94.5, 1, 5);
            }
            // Prime grabber to get stone
            robot.gripBUp();
            sleep(100);
            robot.grabberBDown();
            //calculate angle needed to correct
            correction = robot.correctAngle(90);
            // Straighten out
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .13/correction, 0, 0, 1);
            //approach 2nd stone
            robot.approachStonesBlue(.5);
            //grab 2nd stone
            robot.gripBDown();
            sleep(500);
            robot.grabberBUp();
            // Pull Stone out
            robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 4.5, .6);
            // Correct robot angle
            correction = robot.correctAngle(90) ;
            if (Math.abs(correction) > 3)
                robot.turnPID(90, .13/correction, 0, 0, 1); // Straighten out
            // go to deposit 2nd stone
            if (skyStonePos == "center") {
                robot.goStraightGyro(-90, 1, 7);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(-91, 1, 7);
            } else {
                robot.goStraightGyro(-93, 1, 7);
            }
            robot.strafeLeftGyro(4, .5, 5);
            // grabberR lets go of stone
            robot.grabberBDown();
            robot.gripBUp();
            sleep(200);
            // Grabber folds back up
            robot.grabberBUp();
            robot.gripBDown();
            sleep(200);
            robot.strafeRightGyro(4, .5);
            //park
            if (skyStonePos == "center") {
                robot.goStraightGyro(13, 0.7, 2);
            } else if (skyStonePos == "left") {
                robot.goStraightGyro(13, 0.7, 2);
            } else {
                robot.goStraightGyro(13, 0.7, 2);
            }
            sleep(30000);



            if (skyStonePos == "center") {
                // Move to first skystone
                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(-2, 0.2, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.5); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(600);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 6.5, .6); // Pull Stone out
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

               if (Math.abs(correction) > 3)
                   robot.turnPID(90, .13/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(-60, 1, 7); // go to deposit 1st stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(89.5, 1, 5); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
            //    correction = robot.correctAngle(90) ; //calculate angle needed to correct
           //     if (Math.abs(correction) > 2)
            //        robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.5); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 6.5, .6);
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 3)
                    robot.turnPID(90, .13/correction, 0, 0, 1);
                // Pull Stone out
               // correction = robot.correctAngle(90) ;
                //if (Math.abs(correction) > 2)
                  //  robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(-90, 1, 7); // go to deposit 2nd stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone
                sleep(200);
                robot.grabberBUp();
                robot.gripBDown();
                sleep(200);
                robot.goStraightGyro(13, 0.7, 2); //park
                sleep(30000);
                //robot.alignStonesR(.13);

//                //second stone
//                robot.goStraightGyro(91, .4, 5); // Go back to the stones
//                sleep(500);
//                robot.strafeRightGyro(10.5, .25); // go in to get 2nd stone
//                //robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
//                robot.goStraightGyro(-.6, .2, .2);
//                sleep(1000);
//                robot.strafeLeft(10.5, .5); // Pull stone out
//                robot.goStraightGyro(-100, .7, 7); // Cross the bridge
//               // robot.grabberR.setPosition(.5); // Drop the stone
//                sleep(500);
//                robot.strafeRight(3, .3);
//                robot.goStraightGyro(26, .5, 3); // park
//                sleep(30000);


            } else if (skyStonePos == "left") {

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(-11, 0.2, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.5); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 6.5, .6);
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 3)
                    robot.turnPID(90, .1/correction, 0, 0, 1);
                // Pull Stone out
             //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

               // if (Math.abs(correction) > 2)
                 //   robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(-66, 1, 7); // go to deposit 1st stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(94.5, 1, 5); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
             //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
             //   if (Math.abs(correction) > 2)
               //     robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.5); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 6.5, .6); // Pull Stone out

                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 3)
                    robot.turnPID(90, .1/correction, 0, 0, 1);
             //   correction = robot.correctAngle(90) ;
             //   if (Math.abs(correction) > 2)
             //       robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(-91, 1, 7); // go to deposit 2nd stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(200);
                robot.grabberBUp();
                robot.gripBDown();
                sleep(200);
                robot.goStraightGyro(13, 0.7, 2); //park
                sleep(30000);
            } else { // right

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(3.5, 0.2, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.5); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 6.5, .6); // Pull Stone out
              //  correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

              //  if (Math.abs(correction) > 2)
              //      robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 3)
                    robot.turnPID(90, .1/correction, 0, 0, 1);

                robot.goStraightGyro(-66, 1, 7); // go to deposit 1st stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(94.5, 1, 6); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
                //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
                //   if (Math.abs(correction) > 2)
                //  robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.5); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(robot.encoderAvg() * (11.0/537.6) + 6.5, .6); // Pull Stone out
             //   correction = robot.correctAngle(90) ;
             //   if (Math.abs(correction) > 2)
             //       robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out

                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                // telemetry.addData("start angle: ", startPos.firstAngle);
                // telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);

                //telemetry.update();
                //sleep(5000);

                if (Math.abs(correction) > 3)
                    robot.turnPID(90, .1/correction, 0, 0, 1);

                robot.goStraightGyro(-93, 1, 7); // go to deposit 2nd stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone
                sleep(200);
                robot.grabberBUp();
                robot.gripBDown();
                sleep(200);
                robot.goStraightGyro(13, 0.7, 2); //park
                sleep(30000);
            }
        }
        telemetry.addLine("done");
        telemetry.update();
    }
    }
