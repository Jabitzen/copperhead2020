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
                if (skyStonePos == "center") {
                    robot.goStraightGyro(25, .3, 3); //Move to the stone
                    sleep(500);
                    robot.rotate(90, .4); // Rotate to align grabberR with stone
                    robot.grabberBDown(); //prime grabber facing down towards stones
                    sleep(500);
                    robot.goStraightGyro(1, 0.5, 2); // Align with center stone
                    sleep(30);
                    robot.approachStonesBlue(0.5);//approach stone

                    robot.alignStonesB(.14); //align with stones
                    robot.gripBDown(); // grip stone
                    sleep(1500);
                    robot.grabberBUp(); //bring grabber up to carry stone
                   // robot.goStraightGyro(2, 0.2, 1);
                    sleep(500);
                    robot.strafeRightGyro(6, .5); // Pull Stone out
                    robot.goStraightGyro(-75, .8, 7); // Cross the bridge
                    robot.strafeRightGyro(20, .5); //strafe to foundation
                    robot.grabberBDown();//put grabber down on foundation
                    robot.gripBUp(); //let go of stone
                    robot.grabberBUp(); // grabberR comes back up
                    robot.strafeLeftGyro(20, 0.7, 5); //strafe out away from foundation
                    robot.goStraightGyro(-20, .7, 5); //go to tape to park
                    sleep(30000);

                    robot.goStraightGyro(25, .3, 3); //Move to the stone
                    sleep(500);
                    robot.rotate(84.5, .35); // Rotate to align grabberB with stone
                    sleep(500);
                    robot.approachStonesBlue(9.5); // Approach stone
                    robot.goStraightGyro(1, .3, 1);
                    robot.alignStonesB(.1);
                    robot.grabberBDown(); // Drop grabberB
                    sleep(2000);
                    robot.strafeRightGyro(7, .5); // Pull Stone out
                    // rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .05, .4); // Straighten out
                    robot.goStraightGyro(60, .5, 7); // Cross the bridge
                    robot.grabberBUp(); // grabberB lets go of stone
                    //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .77, .4);
                    //second stone
                    robot.goStraightGyro(-91, .4, 5); // Go back to the stones
                    sleep(500);
                    //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .8, .3);
                    robot.strafeLeft(10.5, .25); // go in to get 2nd stone
                    robot.grabberBDown(); // drop grabberB do hold stone
                    sleep(1000);
                    robot.strafeRightGyro(10.5, .5); // Pull stone out
                    robot.rotate((robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - robot.lastAngles.firstAngle) * .6, .3); // Straighen out
                    robot.goStraightGyro(-100, .7, 7); // Cross the bridge
                    robot.grabberBUp(); // Drop the stone
                    sleep(500);
                    robot.strafeLeft(3, .3);
                    robot.goStraightGyro(26, .5, 3); // park
                    sleep(30000);

                } else if (skyStonePos == "left") {

                    robot.goStraightGyro(25, .3, 3); //Move to the stone
                    sleep(500);
                    robot.rotate(90, .4); // Rotate to align grabberR with stone
                    sleep(500);
                    robot.goStraightGyro(1, 0.5, 2); // Align with center stone
                    sleep(30);
                    robot.approachStonesBlue(0.5);//pproach stone

                    robot.alignStonesB(.14);
                    robot.grabberBDown(); // Drop grabberR
                    sleep(1500);
                    robot.goStraightGyro(2, 0.2, 1);
                    sleep(500);
                    robot.strafeRightGyro(6, .5); // Pull Stone out
                    robot.goStraightGyro(-40, .8, 7); // Cross the bridge
                    robot.grabberBUp(); // grabberR lets go of stone
                    robot.goStraightGyro(13, 0.7, 5);
                    sleep(30000);

                    robot.goStraightGyro(-25, .35, 3); //(-25, .3 + checkDirection(), 0.35); //Move to the stone
                    sleep(500);
                    robot.rotate(-84.5, .35); // Rotate to align grabberB with stone
                    sleep(500);
                    robot.goStraightGyro(-5.2, 0.3, 0.3); // Align with center stone
                    //  sleep(5000);
                    robot.strafeLeft(8.5, .25); // Approach stone
                    robot.grabberBDown(); // Drop grabberB
                    sleep(2000);
                    robot.strafeRightGyro(8.25, .4); // Pull Stone out
                    //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .7, .3); // Straighten out
                    robot.goStraightGyro(-63, .525, 6); // Cross the bridge
                    robot.grabberBUp(); // grabberB lets go of stone
                    //second stone
                    robot.goStraightGyro(95.5, .38, 5); // Go back to the 2nd stones
                    sleep(500);
                    robot.strafeLeft(11.9, .25); // go in to get 2nd stone
                    robot.grabberBDown(); // drop grabberB do hold stone
                    sleep(1000);
                    robot.strafeRightGyro(7.7, .5); // Pull stone out
                    //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .84, .3); // Straighen out
                    robot.goStraightGyro(-112, .7, .79); // Cross the bridge
                    robot.grabberBUp(); // Drop the stone
                    sleep(500);
                    robot.strafeLeft(4, .3); //push stones aside
                    robot.goStraightGyro(30, .5, .59); // park
                    sleep(30000);
                } else { // right
                    robot.goStraightGyro(25, .3, 3); //Move to the stone
                    sleep(500);
                    robot.rotate(90, .4); // Rotate to align grabberR with stone
                    sleep(500);
                    robot.goStraightGyro(1, 0.5, 2); // Align with center stone
                    sleep(30);
                    robot.approachStonesBlue(0.5);//pproach stone

                    robot.alignStonesB(.14);
                    robot.grabberBDown(); // Drop grabberR
                    sleep(1500);
                    robot.goStraightGyro(2, 0.2, 1);
                    sleep(500);
                    robot.strafeRightGyro(6, .5); // Pull Stone out
                    robot.goStraightGyro(-65, .8, 7); // Cross the bridge
                    robot.grabberBUp(); // grabberR lets go of stone
                    robot.goStraightGyro(13, 0.7, 5);
                    sleep(30000);

                    robot.goStraightGyro(-25, .35, 3); //Move to the stone
                    sleep(500);
                    robot.rotate(-84.5, .35); // Rotate to align grabberB with stone
                    sleep(100);
                    robot.goStraightGyro(11.6, 0.3, 3); // Align with right stone
                    //  sleep(5000);
                    robot.strafeLeft(10, .2); // Approach stone
                    robot.grabberBDown(); // Drop grabberB
                    sleep(1900);
                    robot.strafeRightGyro(11, .5); // Pull Stone out
                    //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .75, .3); // Straighten out
                    robot.goStraightGyro(-65, .525, 6); // Cross the bridge
                    robot.grabberBUp(); // grabberB lets go of stone
                    //second stone
                    robot.goStraightGyro(93.45, .415, 7); // Go back to the stones
                    sleep(500);
                    robot.strafeLeft(11.3, .2); // go in to get 2nd stone
                    robot.grabberBDown(); // drop grabberB do hold stone
                    sleep(1000);
                    robot.strafeRightGyro(14, .5); // Pull stone out
                    robot.rotate((robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - robot.lastAngles.firstAngle) * .7, .3); // Straighen out
                    robot.goStraightGyro(-95, .7, 7); // Cross the bridge
                    robot.grabberBUp(); // Drop the stone
                    sleep(500);
                    robot.strafeLeft(3, .3);
                    robot.goStraightGyro(30, .5, 3); // park
                    sleep(30000);
                }
            }
            telemetry.addLine("done");
            telemetry.update();
        }
    }
