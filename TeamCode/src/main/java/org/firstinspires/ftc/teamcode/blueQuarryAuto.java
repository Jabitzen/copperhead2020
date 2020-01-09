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

        robot.init(this);

        waitForStart();

        bm1 = new BitMapVision(this);
        skyStonePos = bm1.findBlueSkystones();
        telemetry.addData("stone", skyStonePos);
        telemetry.update();

        while (opModeIsActive()) {

            // Middle Pathing
            robot.resetAngle();
            //robot.startPos = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            if (skyStonePos == "center") {

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone

                robot.turnPID(90, .79/90, 0, 0, 10); // Rotate to align grabberR with stone
                //sleep(500);
                robot.goStraightGyro(-2, 0.2, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.4); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(8, .6); // Pull Stone out
               // correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

               // if (Math.abs(correction) > 2)
                 //   robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



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
                robot.approachStonesBlue(.4); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(5.5, .6); // Pull Stone out
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
                robot.goStraightGyro(-9, 0.2, 1); // Align with center stone
                robot.gripBUp(); //prime grabber and grip for stone
                robot.grabberBDown();
                sleep(500);
                robot.approachStonesBlue(.4); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(8, .6); // Pull Stone out
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
                robot.approachStonesBlue(.4); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(8, .6); // Pull Stone out
             //   correction = robot.correctAngle(90) ;
             //   if (Math.abs(correction) > 2)
             //       robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(-95, 1, 7); // go to deposit 2nd stone
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
                robot.approachStonesBlue(.4); // Approach stone
                robot.gripBDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(1000);
                robot.grabberBUp(); //pick up stone
                robot.strafeRightGyro(8, .6); // Pull Stone out
              //  correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

              //  if (Math.abs(correction) > 2)
              //      robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out



                //sleep(1000000000);

                robot.goStraightGyro(-59, 1, 7); // go to deposit 1st stone
                robot.grabberBDown();
                robot.gripBUp(); // grabberR lets go of stone

                sleep(300);
                robot.gripBDown(); //put grabber up to go back to quarry
                robot.grabberBUp();
                robot.goStraightGyro(90.5, 1, 6); //go back to quarry
                robot.gripBUp();
                sleep(100);
                robot.grabberBDown();
                //   correction = robot.correctAngle(90) ; //calculate angle needed to correct
                //   if (Math.abs(correction) > 2)
                //  robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.approachStonesBlue(.4); //approach 2nd stone
                robot.gripBDown(); //grab 2nd stone
                sleep(500);
                robot.grabberBUp();
                robot.strafeRightGyro(7.5, .6); // Pull Stone out
             //   correction = robot.correctAngle(90) ;
             //   if (Math.abs(correction) > 2)
             //       robot.turnPID(90, .3/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(-90, 1, 7); // go to deposit 2nd stone
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
