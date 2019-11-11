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

@Autonomous(name="blueQuarryAutoV2", group="12596")
public class blueQuarryAutoV2Merge extends LinearOpMode {

    BitMapVision bm1 = null;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double lastDegrees;
    double globalAngle;
    double referenceAngle;
    String skyStonePos = null;
    RobotHw robot = new RobotHw();

    double leftCorrect;
    double rightCorrect;
    ElapsedTime runtime = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {






        robot.init(this);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        //bm1 = new BitMapVision(this);
        //String skyStonePos = bm1.findSkystones();
        //sleep(10000);
        //telemetry.addData("Skystone Position: ", skyStonePos);
        //telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        //robot.driveTrain.srvMarker.setPosition(1);

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
        waitForStart();

        bm1 = new BitMapVision(this);
        skyStonePos = bm1.findBlueSkystones();
        telemetry.addData("stone", skyStonePos);
        telemetry.update();


        while (opModeIsActive()) {

            // Middle Pathing
            if (skyStonePos == "center") {

                goStraightGyro(-25, .3, 3); //Move to the stone
                sleep(500);
                rotate(-84.5, .35); // Rotate to align grabber with stone
                sleep(500);
                goStraightGyro(-1, 0.3, 2); // Align with center stone
                // sleep(5000);
                robot.strafeLeft(9.5, .25); // Approach stone
                robot.grabberL.setPosition(0.98); // Drop grabber
                sleep(2000);
                strafeRightGyro(7, .5); // Pull Stone out
                // rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .05, .4); // Straighten out
                goStraightGyro(-60, .5, 7); // Cross the bridge
                robot.grabberL.setPosition(.5); // Grabber lets go of stone
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .77, .4);
                //second stone
                goStraightGyro(91, .4, 5); // Go back to the stones
                sleep(500);
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .8, .3);
                robot.strafeLeft(10.5, .25); // go in to get 2nd stone
                robot.grabberL.setPosition(0.98); // drop grabber do hold stone
                sleep(1000);
                strafeRightGyro(10.5, .5); // Pull stone out
                rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .6, .3); // Straighen out
                goStraightGyro(-100, .7, 7); // Cross the bridge
                robot.grabberL.setPosition(.5); // Drop the stone
                sleep(500);
                robot.strafeLeft(3, .3);
                goStraightGyro(26, .5, 3); // park
                sleep(30000);



            } else if (skyStonePos == "left") {
                goStraightGyro(-25, .35, 3); //(-25, .3 + checkDirection(), 0.35); //Move to the stone
                sleep(500);
                rotate(-84.5, .35); // Rotate to align grabber with stone
                sleep(500);
                goStraightGyro(-5.2, 0.3, 0.3); // Align with center stone
                //  sleep(5000);
                robot.strafeLeft(8.5, .25); // Approach stone
                robot.grabberL.setPosition(0.98); // Drop grabber
                sleep(2000);
                strafeRightGyro(8.25, .4); // Pull Stone out
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .7, .3); // Straighten out
                goStraightGyro(-63, .525, 6); // Cross the bridge
                robot.grabberL.setPosition(.5); // Grabber lets go of stone
                //second stone
                goStraightGyro(95.5, .38, 5); // Go back to the 2nd stones
                sleep(500);
                robot.strafeLeft(11.9, .25); // go in to get 2nd stone
                robot.grabberL.setPosition(.98); // drop grabber do hold stone
                sleep(1000);
                strafeRightGyro(7.7, .5); // Pull stone out
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .84, .3); // Straighen out
                robot.goStraight(-112, .7, .79); // Cross the bridge
                robot.grabberL.setPosition(.5); // Drop the stone
                sleep(500);
                robot.strafeLeft(4, .3); //push stones aside
                robot.goStraight(30, .5, .59); // park
                sleep(30000);
            }

            else { // right
                goStraightGyro(-25, .35 , 3); //Move to the stone
                sleep(500);
                rotate(-84.5, .35); // Rotate to align grabber with stone
                sleep(100);
                goStraightGyro(11.6, 0.3 , 3); // Align with right stone
                //  sleep(5000);
                robot.strafeLeft(10, .2); // Approach stone
                robot.grabberL.setPosition(.98); // Drop grabber
                sleep(1900);
                strafeRightGyro(11, .5); // Pull Stone out
                //rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .75, .3); // Straighten out
                goStraightGyro(-65, .525, 6); // Cross the bridge
                robot.grabberL.setPosition(.5); // Grabber lets go of stone
                //second stone
                goStraightGyro(93.45, .415, 7); // Go back to the stones
                sleep(500);
                robot.strafeLeft(11.3, .2); // go in to get 2nd stone
                robot.grabberL.setPosition(.98); // drop grabber do hold stone
                sleep(1000);
                strafeRightGyro(14, .5); // Pull stone out
                rotate((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - lastAngles.firstAngle) * .7, .3); // Straighen out
                goStraightGyro(-95, .7, 7); // Cross the bridge
                robot.grabberL.setPosition(.5); // Drop the stone
                sleep(500);
                robot.strafeLeft(3, .3);
                goStraightGyro(30, .5, 3); // park
                sleep(30000);
            }
        }


        telemetry.addLine("done");
        telemetry.update();

        // String pos = bm1.findSkystones();
        //sleep(3000);
        //telemetry.addData("Skystone Position: ", pos);
       /* telemetry.update();
        sleep(5000);


        if (pos == "left") {
            robot.moveStraight(30, 0.5); //move to skystone
            robot.grabberDown();
            robot.moveStraight(5, .5);
            robot.strafeLeft(60, 0.5);
            robot.grabberUp();
            robot.strafeRight(85, .5);
            robot.grabberDown();
            robot.strafeLeft(85, 0.5);
            robot.strafeRight(20, .5);

        }

        else if (pos == "middle") {

        }

        else if (pos == "right"){

        }

        else {
            telemetry.addLine("oopsie");
        }
        */

    }
    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return -globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.

        double correction, angle, gain = .0;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void correction () {
        // Overshot
        Double newTarget;
        if (getAngle() > lastDegrees) {
            newTarget = lastDegrees - getAngle();
            rotate(newTarget, .15 );
        }
        else {
            newTarget = getAngle() - lastDegrees;
            rotate(newTarget,.15);
        }
    }

    public void gyroCorrect() {
        if (getAngle() > referenceAngle + 1) {
            rightCorrect = .8;
        }
        else if (getAngle() < referenceAngle - 1) {
            leftCorrect = .8;
        }
        else {
            leftCorrect = 1;
            rightCorrect = 1;
        }
    }

    public void setReferenceAngle() {
        resetAngle();
        referenceAngle = getAngle();
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(double degrees, double power)
    {
        /*robot.fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        double  leftPower, rightPower;



        // restart imu movement tracking.
        resetAngle();
        telemetry.addLine().addData("Robot Angle", getAngle());
        sleep(500);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > 0)
        {   // turn right.
            leftPower = power - .13;
            rightPower = -power + .13;
        }
        else if (degrees < 0)
        {   // turn left.
            leftPower = -power + .13;
            rightPower = power - .13;
        }
        else return;


        // set power to rotate.
      /*  robot.fL.setPower(leftPower);
        robot.bL.setPower(leftPower);
        robot.fR.setPower(rightPower);
        robot.bR.setPower(rightPower); */

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                robot.fL.setPower(leftPower);
                robot.bL.setPower(leftPower);
                robot.fR.setPower(rightPower);
                robot.bR.setPower(rightPower);
                telemetry.addData("degrees", getAngle());
                /*telemetry.addData("lastAngle", lastAngles);
                telemetry.addData("globalangle", globalAngle);*/
                telemetry.update();
            }
        }
        else    // right turn.
            while (opModeIsActive() && getAngle() < degrees) {
                robot.fL.setPower(.13 + (leftPower * ((degrees - getAngle())/degrees)));
                robot.bL.setPower(.13 + (leftPower * ((degrees - getAngle())/degrees)));
                robot.fR.setPower(.13 +(-rightPower * ((degrees - getAngle())/degrees)));
                robot.bR.setPower(-.13 - (-rightPower * ((degrees - getAngle())/degrees)));

                telemetry.addData("degrees", getAngle());
                //telemetry.addData("lastangle", lastAngles);
                //telemetry.addData("globalangle", globalAngle);
                telemetry.addData("fl", robot.fL.getPower()) ;
                telemetry.addData("fr", robot.fR.getPower());
                telemetry.addData("bl", robot.bL.getPower());
                telemetry.addData("br", robot.bR.getPower());

                telemetry.update();

                /*if (getAngle() > degrees) {
                    robot.fL.setPower(-leftPower);
                    robot.bL.setPower(-leftPower);
                    robot.fR.setPower(rightPower);
                    robot.bR.setPower(-rightPower);
                }*/
            }

        // turn the motors off.
        robot.stopMotors();
        lastDegrees = degrees;

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        //resetAngle();
    }

    public void straight(double distance, double leftPower, double rightPower) {
        robot.reset();

        double motorPower1;
        double motorPower2;



        // Forward
        if (distance > 0) {
            rightPower = rightPower - .15;
            leftPower = leftPower -.15;

        } // reverse
        else {
            rightPower = -rightPower + .15;
            leftPower = -leftPower + .15;
        }

        double target = Math.abs(distance * (537.6/15.5));

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (distance > 0) {
            while (Math.abs(robot.encoderAvg()) < target && opModeIsActive()) {

                motorPower1 = -.15 + (-rightPower + .15) * ((target - robot.encoderAvg()) / target);
                motorPower2 = .15 + (leftPower * ((target - robot.encoderAvg()) / target));
                robot.fL.setPower(motorPower1);
                robot.fR.setPower(motorPower2);
                robot.bL.setPower(motorPower2);
                robot.bR.setPower(motorPower2);
                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                telemetry.addData("Angle", getAngle());
                telemetry.addData("fl", robot.fL.getPower());
                telemetry.addData("fr", robot.fR.getPower());
                telemetry.addData("bl", robot.bL.getPower());
                telemetry.addData("br", robot.bR.getPower());
                telemetry.update();
            }
        }
        else {
            while (Math.abs(robot.encoderAvg()) < target && opModeIsActive()) {
                motorPower1 = .15 + (-rightPower - .15) * ((target - robot.encoderAvg()) / target);
                motorPower2 = -.15 + (leftPower * ((target - robot.encoderAvg()) / target)) + (getAngle());
                robot.fL.setPower(motorPower1);
                robot.fR.setPower(motorPower2);
                robot.bL.setPower(motorPower2);
                robot.bR.setPower(motorPower2);
                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                telemetry.addData("Angle", getAngle());
                telemetry.addData("fl", robot.fL.getPower());
                telemetry.addData("fr", robot.fR.getPower());
                telemetry.addData("bl", robot.bL.getPower());
                telemetry.addData("br", robot.bR.getPower());
                telemetry.update();
            }
        }
        robot.fL.setPower(0);
        robot.fR.setPower(0);
        robot.bL.setPower(0);
        robot.bR.setPower(0);

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void goStraightGyro(double distance, double leftPower, double timeout) {

        robot.reset();
        resetAngle();
        sleep(100);
        double rightPower;
        // Forward
        if (distance > 0) {
            rightPower = leftPower * 1.45;
            rightPower = rightPower - .15;
            leftPower = leftPower -.15;

        } // reverse
        else {
            rightPower = leftPower * 1.2;
            rightPower = -rightPower + .15;
            leftPower = -leftPower + .15;
        }

        double target = Math.abs(distance * (537.6/15.5));

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runtime.reset();
        if (distance > 0) {
            while (Math.abs(robot.encoderAvg()) < target && opModeIsActive() && runtime.seconds() < timeout) {
                if (getAngle() > 1) {
                    robot.fL.setPower(1.2 * (-.15 + (-rightPower + .15) * ((target - robot.encoderAvg()) / target)));
                    robot.fR.setPower(.8 * (.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bL.setPower(.8 * (.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bR.setPower(1.2 * (.15 + (rightPower * ((target - robot.encoderAvg()) / target))));
                }
                else if (getAngle() < -1) {
                    robot.fL.setPower(.8 * (-.15 + (-rightPower + .15) * ((target - robot.encoderAvg()) / target)));
                    robot.fR.setPower(1.2 * (.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bL.setPower(1.2 * (.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bR.setPower(.8 * (.15 + (rightPower * ((target - robot.encoderAvg()) / target))));
                }
                else {
                    robot.fL.setPower((-.15 + (-rightPower + .15) * ((target - robot.encoderAvg()) / target)));
                    robot.fR.setPower((.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bL.setPower((.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bR.setPower((.15 + (rightPower * ((target - robot.encoderAvg()) / target))));
                }
                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                telemetry.addData("angle", getAngle());
                telemetry.addData("fl", robot.fL.getPower());
                telemetry.addData("fr", robot.fR.getPower());
                telemetry.addData("bl", robot.bL.getPower());
                telemetry.addData("br", robot.bR.getPower());
                telemetry.update();
            }
        }
        else {
            while (Math.abs(robot.encoderAvg()) < target && opModeIsActive() && runtime.seconds() < timeout) {
                if (getAngle() > 1) {
                    robot.fL.setPower(.8 * (.15 + (-rightPower - .15) * ((target - robot.encoderAvg()) / target)));
                    robot.fR.setPower(1.2 * (-.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bL.setPower(1.2 * (-.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bR.setPower(.8 * (-.15 + (rightPower * ((target - robot.encoderAvg()) / target))));
                }
                else if (getAngle() < -1) {
                    robot.fL.setPower(1.2 * (.15 + (-rightPower - .15) * ((target - robot.encoderAvg()) / target)));
                    robot.fR.setPower(.8 * (-.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bL.setPower(.8 * (-.15 + (leftPower * ((target - robot.encoderAvg()) / target))));
                    robot.bR.setPower(1.2 * (-.15 + (rightPower * ((target - robot.encoderAvg()) / target))));
                }
                else {
                    robot.fL.setPower(.15 + (-rightPower - .15) * ((target - robot.encoderAvg()) / target));
                    robot.fR.setPower(-.15 + (leftPower * ((target - robot.encoderAvg()) / target)));
                    robot.bL.setPower(-.15 + (leftPower * ((target - robot.encoderAvg()) / target)));
                    robot.bR.setPower(-.15 + (rightPower * ((target - robot.encoderAvg()) / target)));
                }

                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                telemetry.addData("angle", getAngle());
                telemetry.addData("fl", robot.fL.getPower());
                telemetry.addData("fr", robot.fR.getPower());
                telemetry.addData("bl", robot.bL.getPower());
                telemetry.addData("br", robot.bR.getPower());
                telemetry.update();
            }
        }

        robot.stopMotors();

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        resetAngle();
    }

    public void strafeRightGyro (double distance, double power) {


        robot.reset();
        resetAngle();
        double target = Math.abs(distance * (537.6/11));

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(robot.encoderAvg()) < target && opModeIsActive()) {

            if (getAngle() > 1) {
                robot.fL.setPower(power * .8);
                robot.fR.setPower(power * .8);
                robot.bL.setPower(-power * 1.2);
                robot.bR.setPower(power * 1.2);
            }
            else if (getAngle() < -1) {
                robot.fL.setPower(power * 1.2);
                robot.fR.setPower(power * 1.2);
                robot.bL.setPower(-power * .8);
                robot.bR.setPower(power * .8);
            }
            else {
                robot.fL.setPower(power);
                robot.fR.setPower(power);
                robot.bL.setPower(-power);
                robot.bR.setPower(power);
            }
            //opmode.telemetry.addData("avg", encoderAvg());
            //opmode.telemetry.addData("fl", fL.getCurrentPosition());
            //opmode.telemetry.addData("fr", fR.getCurrentPosition());
            //opmode.telemetry.addData("bl", bL.getCurrentPosition());
            //opmode.telemetry.addData("br", bR.getCurrentPosition());
            telemetry.addData("fl", robot.fL.getPower());
            telemetry.addData("fr", robot.fR.getPower());
            telemetry.addData("bl", robot.bL.getPower());
            telemetry.addData("br", robot.bR.getPower());
            telemetry.update();
        }




        robot.stopMotors();

        robot.fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }
}