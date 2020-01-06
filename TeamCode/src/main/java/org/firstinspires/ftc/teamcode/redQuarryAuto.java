package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;

@Autonomous(name="redQuarryAuto", group="12596")
public class redQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;
    String skyStonePos = null;

    RobotHw robot = new RobotHw();

    double correction = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this);

        waitForStart();

        bm1 = new BitMapVision(this);
        skyStonePos = bm1.findRedSkystones();
        telemetry.addData("stone", skyStonePos);
        telemetry.update();

        while (opModeIsActive()) {

            // Middle Pathing
            robot.resetAngle();
            //robot.startPos = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            if (skyStonePos == "center") {

                robot.goStraightGyro(18, .7, 3); //Move to the 1st stone
                sleep(500);
                robot.turnPID(90, .41/90, 0, 0, 20); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(1, 0.2, 2); // Align with center stone
                robot.gripRUp(); //prime grabber and grip for stone
                robot.grabberRDown();
                sleep(1000);
                robot.approachStonesRed(.4); // Approach stone
                robot.gripRDown(); //grab stone
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);

                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(500);
                robot.grabberRUp(); //pick up stone
                robot.strafeRightGyro(8, .6); // Pull Stone out
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                //telemetry.addData("correct angle : ", correction);
                //telemetry.update();

                if (Math.abs(correction) > 2) {
                    robot.turnPID(correction, .1/correction, 0, 0, 5); // Straighten out
                }


                //sleep(1000000000);

                robot.goStraightGyro(60, .9, 7); // go to deposit 1st stone
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone

                sleep(300);
                robot.gripRDown(); //put grabber up to go back to quarry
                robot.grabberRUp();
                robot.goStraightGyro(-89.5, 0.9, 5); //go back to quarry
                robot.gripRUp();
                robot.grabberRDown();
                sleep(1000);
                correction = robot.correctAngle(90) ; //calculate angle needed to correct
                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out
                robot.approachStonesRed(.4); //approach 2nd stone
                robot.gripRDown(); //grab 2nd stone
                sleep(500);
                robot.grabberRUp();
                robot.strafeRightGyro(10, .6); // Pull Stone out
                correction = robot.correctAngle(90) ;
                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(90, .9, 7); // go to deposit 2nd stone
                robot.grabberRDown();
                robot.gripRUp(); // grabberR lets go of stone

                sleep(500);
                robot.grabberRUp();
                robot.gripRDown();
                sleep(500);
                robot.goStraightGyro(-13, 0.7, 5); //park
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

                robot.goStraightGyro(18, .7, 3); //Move to the stone
                sleep(500);
                robot.turnPID(90, .43/90, 0, 0, 20); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(-1, 0.2, 2); // Align with center stone
                robot.gripRUp();
                robot.grabberRDown();
                sleep(1000);
                robot.approachStonesRed(.4); // Approach stone
                robot.gripRDown();
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);
                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(500);
                robot.strafeRightGyro(11.5, .6); // Pull Stone out
                correction = robot.correctAngle(90) ;
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                telemetry.addData("correct angle : ", correction);
                telemetry.update();
                sleep(1000);

                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out

                //sleep(1000000000);

                robot.goStraightGyro(65, .9, 7); // Cross the bridge
                robot.gripRUp(); // grabberR lets go of stone
                robot.grabberRUp();
                sleep(300);
                robot.gripRDown();
                robot.goStraightGyro(-94.5, 0.9, 5);
                robot.gripRUp();
                robot.grabberRDown();
                sleep(1000);
                correction = robot.correctAngle(90) ;
                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out
                robot.approachStonesRed(.4);
                robot.gripRDown();
                sleep(500);
                robot.strafeRightGyro(12.5, .6); // Pull Stone out
                correction = robot.correctAngle(90) ;
                robot.turnPID(correction, .09/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(95, .9, 7); // Cross the bridge
                robot.gripRUp(); // grabberR lets go of stone
                robot.grabberRUp();
                sleep(300);
                robot.gripRDown();
                robot.goStraightGyro(-13, 0.7, 5);
                sleep(30000);

//                robot.goStraightGyro(25, .3, 3); //Move to the stone
//                sleep(500);
//                robot.rotate(90, .4); // Rotate to align grabberR with stone
//                sleep(500);
//                robot.goStraightGyro(1, 0.5, 2); // Align with center stone
//                sleep(30);
//                robot.approachStonesRed(5); // Approach stone
//
//                robot.alignStonesR(.14);
//                robot.grabberR.setPosition(0.65); // Drop grabberR
//                sleep(1500);
//                robot.goStraightGyro(-4, 0.2, 1);
//                sleep(500);
//                robot.strafeRightGyro(7, .5); // Pull Stone out
//                robot.goStraightGyro(70, .8, 7); // Cross the bridge
//                robot.grabberR.setPosition(.2); // grabberR lets go of stone
//                robot.goStraightGyro(-13, 0.7, 5);
//                sleep(30000);
//
//                robot.goStraightGyro(-25, .35, 3); //(-25, .3 + checkDirection(), 0.35); //Move to the stone
//                sleep(500);
//                robot.rotate(84.5, .35); // Rotate to align grabberR with stone
//                sleep(500);
//                robot.goStraightGyro(5.2, 0.3, 0.3); // Align with center stone
//                //  sleep(5000);
//                robot.strafeRightGyro(8.5, .25); // Approach stone
//                robot.grabberR.setPosition(0.02); // Drop grabberR
//                robot.goStraightGyro(-.6, .2, .2);
//
//                sleep(2000);
//                robot.strafeLeft(8.25, .4); // Pull Stone out
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .7, .3); // Straighten out
//                robot.goStraightGyro(-63, .525, 6); // Cross the bridge
//                robot.grabberR.setPosition(.5); // grabberR lets go of stone
//                //second stone
//                robot.goStraightGyro(95.5, .38, 5); // Go back to the 2nd stones
//                sleep(500);
//                robot.strafeRightGyro(11.9, .25); // go in to get 2nd stone
//                robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
//                sleep(1000);
//                robot.strafeLeft(7.7, .5); // Pull stone out
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .84, .3); // Straighen out
//                robot.goStraightGyro(-112, .7, .79); // Cross the bridge
//                robot.grabberR.setPosition(.5); // Drop the stone
//                sleep(500);
//                robot.strafeRightGyro(4, .3); //push stones aside
//                robot.goStraightGyro(30, .5, .59); // park
//                sleep(30000);
            } else { // right

                robot.goStraightGyro(18, .7, 3); //Move to the stone
                sleep(500);
                robot.turnPID(90, .43/90, 0, 0, 20); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(3, 0.2, 2); // Align with center stone
                robot.gripRUp();
                robot.grabberRDown();
                sleep(1000);
                robot.approachStonesRed(.4); // Approach stone
                robot.gripRDown();
                //robot.alignStonesR(.14);
                //robot.grabberR.setPosition(0.65); // Drop grabberR
                //sleep(1500);
                //robot.goStraightGyro(-4, 0.2, 1);
                sleep(500);
                robot.strafeRightGyro(11.5, .6); // Pull Stone out
                correction = robot.correctAngle(90) ;
//                telemetry.addData("start angle: ", startPos.firstAngle);
//                telemetry.addData("angle : ", robot.getAngle());
                telemetry.addData("correct angle : ", correction);
                telemetry.update();
                sleep(1000);

                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out

                //sleep(1000000000);

                robot.goStraightGyro(55, .9, 7); // Cross the bridge
                robot.gripRUp(); // grabberR lets go of stone
                robot.grabberRUp();
                sleep(300);
                robot.gripRDown();
                robot.goStraightGyro(-84.5, 0.9, 5);
                robot.gripRUp();
                robot.grabberRDown();
                sleep(1000);
                correction = robot.correctAngle(90) ;
                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out
                robot.approachStonesRed(.4);
                robot.gripRDown();
                sleep(500);
                robot.strafeRightGyro(12.5, .6); // Pull Stone out
                correction = robot.correctAngle(90) ;
                robot.turnPID(correction, .1/correction, 0, 0, 1); // Straighten out
                robot.goStraightGyro(85, .9, 7); // Cross the bridge
                robot.gripRUp(); // grabberR lets go of stone
                robot.grabberRUp();
                sleep(300);
                robot.gripRDown();
                robot.goStraightGyro(-13, 0.7, 5);
                sleep(30000);

//                robot.goStraightGyro(25, .3, 3); //Move to the stone
//                sleep(500);
//                robot.rotate(90, .4); // Rotate to align grabberR with stone
//                sleep(500);
//                robot.goStraightGyro(1, 0.5, 2); // Align with center stone
//                sleep(30);
//                robot.approachStonesRed(5); // Approach stone
//
//                robot.alignStonesR(.14);
//                robot.grabberR.setPosition(0.65); // Drop grabberR
//                sleep(1500);
//                robot.goStraightGyro(-4, 0.2, 1);
//                sleep(500);
//                robot.strafeRightGyro(7, .5); // Pull Stone out
//                robot.goStraightGyro(48, .8, 7); // Cross the bridge
//                robot.grabberR.setPosition(.2); // grabberR lets go of stone
//                robot.goStraightGyro(-13, 0.7, 5);
//                sleep(30000);
//
//                robot.goStraightGyro(-25, .35, 3); //Move to the stone
//                sleep(500);
//                robot.rotate(84.5, .35); // Rotate to align grabberR with stone    //84.5
//                sleep(100);
//                robot.goStraightGyro(-11.6, 0.3, 3); // Align with right stone
//                //sleep(5000);
//                robot.strafeRightGyro(8.5, .2); // Approach stone //10
//                robot.grabberR.setPosition(0.02); // Drop grabberR
//                //EXTRA ALIGN: robot.goStraightGyro(-.6, .2, .2);
//
//                //sleep(5000);
//                robot.strafeLeft(8.5, .5); // Pull Stone out //11
//                robot.rotate((robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - robot.lastAngles.firstAngle) * .75, .3); // Straighten out
//                robot.goStraightGyro(-65, .525, 6); // Cross the bridge
//                robot.grabberR.setPosition(.5); // grabberR lets go of stone
//                //second stone
//                robot.goStraightGyro(93.45, .415, 7); // Go back to the stones
//                sleep(500);
//                robot.strafeRightGyro(11.3, .2); // go in to get 2nd stone
//                robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
//                robot.goStraightGyro(-.6, .2, .2);
//                sleep(1000);
//                robot.strafeLeft(14, .5); // Pull stone out
//                robot.goStraightGyro(-95, .7, 7); // Cross the bridge
//                robot.grabberR.setPosition(.5); // Drop the stone
//                robot.strafeRightGyro(3, .3);
//                robot.goStraightGyro(30, .5, 3); // park
//                sleep(30000);
            }
        }
        telemetry.addLine("done");
        telemetry.update();
    }


}