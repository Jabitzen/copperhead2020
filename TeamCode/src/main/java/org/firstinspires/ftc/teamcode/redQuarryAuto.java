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
            //NEW PATH
//            robot.goStraightGyro(-25, .3, 3); //drive towards stones
//            robot.rotate(90, .3); //rotate to align grabberR with stines
//            robot.approachStones(.3); //strafe to stones
//            robot.alignWithStones(.3); //aligns with stones
//            robot.grabberR.setPosition(0.02); //grab stone
//            robot.strafeRightGyro(6, .3); //pull stone out
//            robot.goStraightGyro(-60, .3, 5); //drive with stone across bridge
//            robot.grabberR.setPosition(.98); //release stone
//            sleep(2000);
//            robot.goStraightGyro(30, .3, 4); //drive back to second stone
//            robot.approachStones(.3); //strafe to second stone
//            robot.alignWithStones(.3); //aligns with 2nd stone
//            robot.grabberR.setPosition(0.02); //grab 2nd stone
//            robot.strafeRightGyro(6, .3); //pull 2nd stone out
//            robot.goStraightGyro(-60, .3, 5); //pull second stone across bridge
//            sleep(1000);
//            robot.grabberR.setPosition(.98); //release 2nd stone
//            robot.goStraightGyro(5, .3, 2); //park


            if (skyStonePos == "center") {

                robot.goStraightGyro(25, .3, 3); //Move to the stone
                sleep(500);
                robot.rotate(90, .4); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(1, 0.5, 2); // Align with center stone
                sleep(30);
                robot.approachStonesRed(5); // Approach stone

                robot.alignStonesR(.14);
                robot.grabberR.setPosition(0.65); // Drop grabberR
                sleep(1500);
                robot.goStraightGyro(-4, 0.2, 1);
                sleep(500);
                robot.strafeRightGyro(7, .5); // Pull Stone out
                robot.goStraightGyro(55, .8, 7); // Cross the bridge
                robot.grabberR.setPosition(.2); // grabberR lets go of stone
                robot.goStraightGyro(-13, 0.7, 5);
                sleep(30000);

                //second stone
                robot.goStraightGyro(91, .4, 5); // Go back to the stones
                sleep(500);
                robot.strafeRightGyro(10.5, .25); // go in to get 2nd stone
                //robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
                robot.goStraightGyro(-.6, .2, .2);
                sleep(1000);
                robot.strafeLeft(10.5, .5); // Pull stone out
                robot.goStraightGyro(-100, .7, 7); // Cross the bridge
              //  robot.grabberR.setPosition(.5); // Drop the stone
                sleep(500);
                robot.strafeRight(3, .3);
                robot.goStraightGyro(26, .5, 3); // park
                sleep(30000);


            } else if (skyStonePos == "left") {

                robot.goStraightGyro(25, .3, 3); //Move to the stone
                sleep(500);
                robot.rotate(90, .4); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(1, 0.5, 2); // Align with center stone
                sleep(30);
                robot.approachStonesRed(5); // Approach stone

                robot.alignStonesR(.14);
                robot.grabberR.setPosition(0.65); // Drop grabberR
                sleep(1500);
                robot.goStraightGyro(-4, 0.2, 1);
                sleep(500);
                robot.strafeRightGyro(7, .5); // Pull Stone out
                robot.goStraightGyro(70, .8, 7); // Cross the bridge
                robot.grabberR.setPosition(.2); // grabberR lets go of stone
                robot.goStraightGyro(-13, 0.7, 5);
                sleep(30000);

                robot.goStraightGyro(-25, .35, 3); //(-25, .3 + checkDirection(), 0.35); //Move to the stone
                sleep(500);
                robot.rotate(84.5, .35); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(5.2, 0.3, 0.3); // Align with center stone
                //  sleep(5000);
                robot.strafeRightGyro(8.5, .25); // Approach stone
                robot.grabberR.setPosition(0.02); // Drop grabberR
                robot.goStraightGyro(-.6, .2, .2);

                sleep(2000);
                robot.strafeLeft(8.25, .4); // Pull Stone out
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .7, .3); // Straighten out
                robot.goStraightGyro(-63, .525, 6); // Cross the bridge
                robot.grabberR.setPosition(.5); // grabberR lets go of stone
                //second stone
                robot.goStraightGyro(95.5, .38, 5); // Go back to the 2nd stones
                sleep(500);
                robot.strafeRightGyro(11.9, .25); // go in to get 2nd stone
                robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
                sleep(1000);
                robot.strafeLeft(7.7, .5); // Pull stone out
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .84, .3); // Straighen out
                robot.goStraightGyro(-112, .7, .79); // Cross the bridge
                robot.grabberR.setPosition(.5); // Drop the stone
                sleep(500);
                robot.strafeRightGyro(4, .3); //push stones aside
                robot.goStraightGyro(30, .5, .59); // park
                sleep(30000);
            } else { // right

                robot.goStraightGyro(25, .3, 3); //Move to the stone
                sleep(500);
                robot.rotate(90, .4); // Rotate to align grabberR with stone
                sleep(500);
                robot.goStraightGyro(1, 0.5, 2); // Align with center stone
                sleep(30);
                robot.approachStonesRed(5); // Approach stone

                robot.alignStonesR(.14);
                robot.grabberR.setPosition(0.65); // Drop grabberR
                sleep(1500);
                robot.goStraightGyro(-4, 0.2, 1);
                sleep(500);
                robot.strafeRightGyro(7, .5); // Pull Stone out
                robot.goStraightGyro(48, .8, 7); // Cross the bridge
                robot.grabberR.setPosition(.2); // grabberR lets go of stone
                robot.goStraightGyro(-13, 0.7, 5);
                sleep(30000);

                robot.goStraightGyro(-25, .35, 3); //Move to the stone
                sleep(500);
                robot.rotate(84.5, .35); // Rotate to align grabberR with stone    //84.5
                sleep(100);
                robot.goStraightGyro(-11.6, 0.3, 3); // Align with right stone
                //sleep(5000);
                robot.strafeRightGyro(8.5, .2); // Approach stone //10
                robot.grabberR.setPosition(0.02); // Drop grabberR
                //EXTRA ALIGN: robot.goStraightGyro(-.6, .2, .2);

                //sleep(5000);
                robot.strafeLeft(8.5, .5); // Pull Stone out //11
                robot.rotate((robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - robot.lastAngles.firstAngle) * .75, .3); // Straighten out
                robot.goStraightGyro(-65, .525, 6); // Cross the bridge
                robot.grabberR.setPosition(.5); // grabberR lets go of stone
                //second stone
                robot.goStraightGyro(93.45, .415, 7); // Go back to the stones
                sleep(500);
                robot.strafeRightGyro(11.3, .2); // go in to get 2nd stone
                robot.grabberR.setPosition(0.02); // drop grabberR do hold stone
                robot.goStraightGyro(-.6, .2, .2);
                sleep(1000);
                robot.strafeLeft(14, .5); // Pull stone out
                robot.goStraightGyro(-95, .7, 7); // Cross the bridge
                robot.grabberR.setPosition(.5); // Drop the stone
                robot.strafeRightGyro(3, .3);
                robot.goStraightGyro(30, .5, 3); // park
                sleep(30000);
            }
        }


        telemetry.addLine("done");
        telemetry.update();
    }
}