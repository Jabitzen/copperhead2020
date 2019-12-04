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
    //BNO055IMU               imu;
    //Orientation             lastAngles = new Orientation();
  //  double lastDegrees;
    //double globalAngle;
    //double referenceAngle;
/*
    ColorSensor sensorColorBotBack;
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorRight;
    DistanceSensor sensorDistanceBotBack;
    DistanceSensor sensorDistanceLeft;
    DistanceSensor sensorDistanceRight;
*/

    String skyStonePos = null;
    RobotHw robot = new RobotHw();

   // double leftCorrect;
    //double rightCorrect;
    //ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(this);
        // BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //  parameters.mode                = BNO055IMU.SensorMode.IMU;
        // parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        // parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        bm1 = new BitMapVision(this);
        skyStonePos = bm1.findBlueSkystones();
        telemetry.addData("stone", skyStonePos);
        telemetry.update();
        //sleep(10000);
        //telemetry.addData("Skystone Position: ", skyStonePos);
        //telemetry.update();

        //  imu = hardwareMap.get(BNO055IMU.class, "imu");

        //imu.initialize(parameters);
        //initColor();
        //robot.driveTrain.srvMarker.setPosition(1);


        /*


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated() )
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //skyStonePos = "left";
        /*
         */
        waitForStart();

        while (opModeIsActive()) {
            robot.goStraightGyro(20, .3, 5);
            sleep(500);
            robot.rotate(90, .35);
            sleep(500);
            robot.approachStonesBlue(.2);
            // telemetry.addData("edge", robot.sensorDistanceBEdge.getDistance(DistanceUnit.CM));
            sleep(1000);
            robot.alignStonesB(0.2);
            telemetry.addLine("done 2");
            telemetry.update();
            robot.grabber.setPosition(0.98);
            robot.strafeRightGyro(7, 0.5);
            robot.goStraightGyro(-60, 0.5, 7);
            robot.grabber.setPosition(0.5);
            robot.goStraightGyro(30, 0.5, 3);
            sleep(30000);
        }


//        bm1 = new BitMapVision(this);
//        skyStonePos = bm1.findBlueSkystones();
//        telemetry.addData("stone", skyStonePos);
//        telemetry.update();
//
//
//        while (opModeIsActive()) {
//
//            // Middle Pathing
//            if (skyStonePos == "center") {
//
//                robot.goStraightGyro(-25, .3, 3); //Move to the stone
//                sleep(500);
//                robot.rotate(-84.5, .35); // Rotate to align grabber with stone
//                sleep(500);
//                robot.goStraightGyro(-1, 0.3, 2); // Align with center stone
//                // sleep(5000);
//                robot.strafeLeft(9.5, .25); // Approach stone
//                robot.grabber.setPosition(0.98); // Drop grabber
//                sleep(2000);
//                robot.strafeRightGyro(7, .5); // Pull Stone out
//                // rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .05, .4); // Straighten out
//                robot.goStraightGyro(-60, .5, 7); // Cross the bridge
//                robot.grabber.setPosition(.5); // Grabber lets go of stone
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .77, .4);
//                //second stone
//                robot.goStraightGyro(91, .4, 5); // Go back to the stones
//                sleep(500);
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .8, .3);
//                robot.strafeLeft(10.5, .25); // go in to get 2nd stone
//                robot.grabber.setPosition(0.98); // drop grabber do hold stone
//                sleep(1000);
//                robot.strafeRightGyro(10.5, .5); // Pull stone out
//                robot.rotate((robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - robot.lastAngles.firstAngle) * .6, .3); // Straighen out
//                robot.goStraightGyro(-100, .7, 7); // Cross the bridge
//                robot.grabber.setPosition(.5); // Drop the stone
//                sleep(500);
//                robot.strafeLeft(3, .3);
//                robot.goStraightGyro(26, .5, 3); // park
//                sleep(30000);
//
//
//
//            } else if (skyStonePos == "left") {
//                robot.goStraightGyro(-25, .35, 3); //(-25, .3 + checkDirection(), 0.35); //Move to the stone
//                sleep(500);
//                robot.rotate(-84.5, .35); // Rotate to align grabber with stone
//                sleep(500);
//                robot.goStraightGyro(-5.2, 0.3, 0.3); // Align with center stone
//                //  sleep(5000);
//                robot.strafeLeft(8.5, .25); // Approach stone
//                robot.grabber.setPosition(0.98); // Drop grabber
//                sleep(2000);
//                robot.strafeRightGyro(8.25, .4); // Pull Stone out
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .7, .3); // Straighten out
//                robot.goStraightGyro(-63, .525, 6); // Cross the bridge
//                robot.grabber.setPosition(.5); // Grabber lets go of stone
//                //second stone
//                robot.goStraightGyro(95.5, .38, 5); // Go back to the 2nd stones
//                sleep(500);
//                robot.strafeLeft(11.9, .25); // go in to get 2nd stone
//                robot.grabber.setPosition(.98); // drop grabber do hold stone
//                sleep(1000);
//                robot.strafeRightGyro(7.7, .5); // Pull stone out
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .84, .3); // Straighen out
//                robot.goStraightGyro(-112, .7, .79); // Cross the bridge
//                robot.grabber.setPosition(.5); // Drop the stone
//                sleep(500);
//                robot.strafeLeft(4, .3); //push stones aside
//                robot.goStraightGyro(30, .5, .59); // park
//                sleep(30000);
//            }
//
//            else { // right
//                robot.goStraightGyro(-25, .35 , 3); //Move to the stone
//                sleep(500);
//                robot.rotate(-84.5, .35); // Rotate to align grabber with stone
//                sleep(100);
//                robot.goStraightGyro(11.6, 0.3 , 3); // Align with right stone
//                //  sleep(5000);
//                robot.strafeLeft(10, .2); // Approach stone
//                robot.grabber.setPosition(.98); // Drop grabber
//                sleep(1900);
//                robot.strafeRightGyro(11, .5); // Pull Stone out
//                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .75, .3); // Straighten out
//                robot.goStraightGyro(-65, .525, 6); // Cross the bridge
//                robot.grabber.setPosition(.5); // Grabber lets go of stone
//                //second stone
//                robot.goStraightGyro(93.45, .415, 7); // Go back to the stones
//                sleep(500);
//                robot.strafeLeft(11.3, .2); // go in to get 2nd stone
//                robot.grabber.setPosition(.98); // drop grabber do hold stone
//                sleep(1000);
//                robot.strafeRightGyro(14, .5); // Pull stone out
//                robot.rotate((robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - robot.lastAngles.firstAngle) * .7, .3); // Straighen out
//                robot.goStraightGyro(-95, .7, 7); // Cross the bridge
//                robot.grabber.setPosition(.5); // Drop the stone
//                sleep(500);
//                robot.strafeLeft(3, .3);
//                robot.goStraightGyro(30, .5, 3); // park
//                sleep(30000);
//            }
//        }


        telemetry.addLine("done");
        telemetry.update();

    }

}