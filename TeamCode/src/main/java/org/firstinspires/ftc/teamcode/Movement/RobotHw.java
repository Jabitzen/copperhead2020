package org.firstinspires.ftc.teamcode.Movement;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotHw {

    // Motors
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;
    public DcMotor intakeR = null;
    public DcMotor intakeL = null;
    public DcMotor lift = null;
    public DcMotor rotateMotor = null;

    // Servos
    public Servo clip = null;
    public Servo claw = null;
    public Servo rotate = null;
    public Servo clamp = null;
    public Servo grabberL = null;
    public Servo grabberR = null;

    // HardwareMap
    HardwareMap hwMap;

    // Linear Opmode
    LinearOpMode opmode;

    // Time
    public ElapsedTime runtime = new ElapsedTime();

    // Tick Conversion
    static final double COUNTS_PER_MOTOR_REV = 380;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double lastDegrees;
    public double globalAngle;
    public double referenceAngle;

    public double leftCorrect;
    public double rightCorrect;

    public ColorSensor sensorColorBotBack;
    public ColorSensor sensorColorLeft;
    public ColorSensor sensorColorRight;
    public DistanceSensor sensorDistanceBotBack;
    public DistanceSensor sensorDistanceLeft;
    public DistanceSensor sensorDistanceRight;


    // Initialize Components
    public void init(LinearOpMode lOpmode) {
        opmode = lOpmode;
        // Hardware map
        hwMap = opmode.hardwareMap;

        // Define and Initialize Motors
        fL = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR = opmode.hardwareMap.get(DcMotor.class, "bR");

        //intakeR  = hwMap.get(DcMotor.class, "intakeR");
        //intakeL  = hwMap.get(DcMotor.class, "intakeL");

        //lift = hwMap.get(DcMotor.class, "lift");
        //rotateMotor = hwMap.get(DcMotor.class, "rotateMotor");

        //Define and initialize servos
        clip = hwMap.get(Servo.class, "clip");
        claw = hwMap.get(Servo.class, "claw");
        rotate = hwMap.get(Servo.class, "rotate");
        clamp = hwMap.get(Servo.class, "clamp");
        grabberL = hwMap.get(Servo.class, "grabberL");
        grabberR = hwMap.get(Servo.class, "grabberR");

        //set direction of motors
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        initColor();
        //intakeL.setDirection(DcMotor.Direction.FORWARD);
        //intakeR.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders, and run without encoders
        reset();

        // Set motor powers to zero
        stopMotors();


        // Set all motors to zero power
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        //driveTrain.srvMarker.setPosition(1);

        opmode.telemetry.addData("Mode", "calibrating...");
        opmode.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!opmode.isStopRequested() && !imu.isGyroCalibrated()) {
            opmode.sleep(50);
            opmode.idle();
        }

        opmode.telemetry.addData("Mode", "waiting for start");
        opmode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        opmode.telemetry.update();

        opmode.telemetry.addData("fl", fL.getCurrentPosition());
        opmode.telemetry.addData("fr", fR.getCurrentPosition());
        opmode.telemetry.addData("bl", bL.getCurrentPosition());
        opmode.telemetry.addData("br", bR.getCurrentPosition());
        opmode.telemetry.update();

    }

    public double atTarget(double distance) {
        return Math.abs(distance * COUNTS_PER_INCH);
    }

    public void moveStraight(double distance, double power) {
        dtEncoderModeOn();
        int startVal = fL.getCurrentPosition();
        if (fL.getCurrentPosition() != 0) {
            while (Math.abs(fL.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        } else if (fR.getCurrentPosition() != 0) {
            while (Math.abs(fR.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        } else if (bL.getCurrentPosition() != 0) {
            while (Math.abs(bL.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        } else if (bR.getCurrentPosition() != 0) {
            while (Math.abs(bR.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        }
    }

    // Set motors to zero power
    public void stopMotors() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    // Strafe right
    public void strafeRight(double distance, double power) {
        // Reset encoders
        reset();
        // Set desired target position
        double target = Math.abs(distance * (537.6 / 11));
        // Set motors to brake
        brakeMode();
        // While robot is moving to desired position
        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
            /* debugging code, uncomment if needed
            opmode.telemetry.addData("avg", encoderAvg());
            opmode.telemetry.addData("fl", fL.getCurrentPosition());
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
            */
        }
        // Set motors to zero power
        stopMotors();
        // Set motors to float
        floatMode();
    }

    // Strafe left
    public void strafeLeft(double distance, double power) {
        // Reset encoders
        reset();
        // Set desired target position
        double target = Math.abs(distance * (537.6 / 11));
        // Set motors to brake
        brakeMode();
        // While robot is moving to desired position
        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);
            /* debugging code, uncomment if needed
            opmode.telemetry.addData("avg", encoderAvg());
            opmode.telemetry.addData("fl", fL.getCurrentPosition());
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
            */
        }
        // Set motors power to 0
        stopMotors();
        // Set motors to float
        floatMode();
    }

    // Reset Encoders, and set mode to run without encoders
    public void reset() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opmode.idle();
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opmode.idle();
    }

    // Return avg of all 4 motor encoder values
    public double encoderAvg() {
        double avg = 0;
        // FR motor
        avg += Math.abs(fR.getCurrentPosition());
        // FL motor
        avg += Math.abs(fL.getCurrentPosition());
        //BL motor
        avg += Math.abs(bL.getCurrentPosition());
        //BR motor
        avg += Math.abs(bR.getCurrentPosition());
        return avg / 4;
    }

    // Turn on encoders
    public void dtEncoderModeOn() {
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set motors to freely rotate at zero power
    public void floatMode() {
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Set motors to hold position at zero power
    public void brakeMode() {
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void goStraightGyro(double distance, double leftPower, double timeout) {
        fL.setDirection(DcMotor.Direction.FORWARD);
        reset();
        resetAngle();
        opmode.sleep(100);
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

        brakeMode();
        runtime.reset();
        if (distance > 0) {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < timeout) {
                if (getAngle() > 1) {
                    fL.setPower(.8 * (-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(1.2 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(1.2 * (.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else if (getAngle() < -1) {
                    fL.setPower(1.2 * (-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(1.2 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(.8 * (.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else {
                    fL.setPower((-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower((.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                /*
                opmode.telemetry.addData("angle", getAngle());
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
                */

            }
        }
        else {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive() && runtime.seconds() < timeout) {
                if (getAngle() > 1) {
                    fL.setPower(1.2 * (.15 + (-rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(1.2 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(.8 * (-.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else if (getAngle() < -1) {
                    fL.setPower(.8 * (.15 + (-rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(1.2 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(1.2 * (-.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else {
                    fL.setPower(.15 + (-rightPower - .15) * ((target - encoderAvg()) / target));
                    fR.setPower(-.15 + (leftPower * ((target - encoderAvg()) / target)));
                    bL.setPower(-.15 + (leftPower * ((target - encoderAvg()) / target)));
                    bR.setPower(-.15 + (rightPower * ((target - encoderAvg()) / target)));
                }

                //opmode.telemetry.addData("avg", encoderAvg());
                //opmode.telemetry.addData("fl", fL.getCurrentPosition());
                //opmode.telemetry.addData("fr", fR.getCurrentPosition());
                //opmode.telemetry.addData("bl", bL.getCurrentPosition());
                //opmode.telemetry.addData("br", bR.getCurrentPosition());
                opmode.telemetry.addData("angle", getAngle());
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        }

        stopMotors();
        fL.setDirection(DcMotor.Direction.REVERSE);
        floatMode();
        resetAngle();
    }


    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
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

    public double checkDirection() {
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


    public void rotate(double degrees, double power) {
        fR.setDirection(DcMotor.Direction.FORWARD);

        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftPower, rightPower;


        // restart imu movement tracking.
        resetAngle();
        opmode.telemetry.addLine().addData("Robot Angle", getAngle());
        opmode.sleep(500);


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > 0) {   // turn right.
            leftPower = power - .13;
            rightPower = -power + .13;
        } else if (degrees < 0) {   // turn left.
            leftPower = -power + .13;
            rightPower = power - .13;
        } else return;


        // set power to rotate.
        fL.setPower(leftPower);
        bL.setPower(leftPower);
        fR.setPower(rightPower);
        bR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.

            while (opmode.opModeIsActive() && getAngle() > degrees) {
                fL.setPower(-.13 + (leftPower * ((degrees - getAngle()) / degrees)));
                bL.setPower(-.13 + (leftPower * ((degrees - getAngle()) / degrees)));
                fR.setPower(-.13 + (-rightPower * ((degrees - getAngle()) / degrees)));
                bR.setPower(.13 - (-rightPower * ((degrees - getAngle()) / degrees)));

                opmode.telemetry.addData("degrees", getAngle());
                opmode.telemetry.addData("lastAngle", lastAngles);
                opmode.telemetry.addData("globalangle", globalAngle);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        } else    // right turn.
            while (opmode.opModeIsActive() && getAngle() < degrees) {
                fL.setPower(.13 + (leftPower * ((degrees - getAngle()) / degrees)));
                bL.setPower(.13 + (leftPower * ((degrees - getAngle()) / degrees)));
                fR.setPower(.13 + (-rightPower * ((degrees - getAngle()) / degrees)));
                bR.setPower(-.13 - (-rightPower * ((degrees - getAngle()) / degrees)));

                opmode.telemetry.addData("degrees", getAngle());
                //telemetry.addData("lastangle", lastAngles);
                //telemetry.addData("globalangle", globalAngle);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());

                opmode.telemetry.update();

                if (getAngle() > degrees) {
                    fL.setPower(-leftPower);
                    bL.setPower(-leftPower);
                    fR.setPower(rightPower);
                    bR.setPower(-rightPower);
                }
            }

 //turn the motors off.
        fR.setDirection(DcMotor.Direction.REVERSE);
        stopMotors();
        lastDegrees = degrees;

        // wait for rotation to stop.
        opmode.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
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

    public void strafeRightGyro (double distance, double power) {


        reset();
        resetAngle();
        double target = Math.abs(distance * (537.6/11));

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {

            if (getAngle() > 1) {
                fL.setPower(power * .8);
                fR.setPower(-power * .8);
                bL.setPower(-power * 1.2);
                bR.setPower(power * 1.2);
            }
            else if (getAngle() < -1) {
                fL.setPower(power * 1.2);
                fR.setPower(-power * 1.2);
                bL.setPower(-power * .8);
                bR.setPower(power * .8);
            }
            else {
                fL.setPower(power);
                fR.setPower(-power);
                bL.setPower(-power);
                bR.setPower(power);
            }
            //opmode.telemetry.addData("avg", encoderAvg());
            //opmode.telemetry.addData("fl", fL.getCurrentPosition());
            //opmode.telemetry.addData("fr", fR.getCurrentPosition());
            //opmode.telemetry.addData("bl", bL.getCurrentPosition());
            //opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
        }
        stopMotors();

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void strafeLeftGyro (double distance, double power) {
        reset();
        resetAngle();
        double target = Math.abs(distance * (537.6/11));

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {

            if (getAngle() > 1) {
                fL.setPower(-power * 1.2);
                fR.setPower(power * 1.2);
                bL.setPower(power * .8);
                bR.setPower(-power * .8);
            }
            else if (getAngle() < -1) {
                fL.setPower(-power * .8);
                fR.setPower(power * .8);
                bL.setPower(power * 1.2);
                bR.setPower(-power * 1.2);
            }
            else {
                fL.setPower(-power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(-power);
            }
            //opmode.telemetry.addData("avg", encoderAvg());
            //opmode.telemetry.addData("fl", fL.getCurrentPosition());
            //opmode.telemetry.addData("fr", fR.getCurrentPosition());
            //opmode.telemetry.addData("bl", bL.getCurrentPosition());
            //opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.addData("fl", fL.getPower());
            opmode.telemetry.addData("fr", fR.getPower());
            opmode.telemetry.addData("bl", bL.getPower());
            opmode.telemetry.addData("br", bR.getPower());
            opmode.telemetry.update();
        }
        stopMotors();

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initColor() {

        sensorColorBotBack = hwMap.get(ColorSensor.class, "sensorColorBotFront");
        sensorColorLeft = hwMap.get(ColorSensor.class, "sensorColorLeft");
        sensorColorRight = hwMap.get(ColorSensor.class, "sensorColorRight");

        // get a reference to the distance sensor that shares the same name.
        sensorDistanceLeft = hwMap.get(DistanceSensor.class, "sensorColorLeft");
        sensorDistanceRight = hwMap.get(DistanceSensor.class, "sensorColorRight");
        sensorDistanceBotBack = hwMap.get(DistanceSensor.class, "sensorColorBotFront");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;
        sensorColorLeft.enableLed(false);
        sensorColorRight.enableLed(false);
        sensorColorBotBack.enableLed(false);

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);

        while (opmode.opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColorLeft.red() * SCALE_FACTOR),
                    (int) (sensorColorLeft.green() * SCALE_FACTOR),
                    (int) (sensorColorLeft.blue() * SCALE_FACTOR),
                    hsvValues);
            Color.RGBToHSV((int) (sensorColorBotBack.red() * SCALE_FACTOR),
                    (int) (sensorColorBotBack.green() * SCALE_FACTOR),
                    (int) (sensorColorBotBack.blue() * SCALE_FACTOR),
                    hsvValues);
        }
    }

    public void moveLeft(double power) {
        if (getAngle() > 1) {
            fL.setPower(-power * 1.2);
            fR.setPower(power * 1.2);
            bL.setPower(power * .8);
            bR.setPower(-power * .8);
        }
        else if (getAngle() < -1) {
            fL.setPower(-power * .8);
            fR.setPower(power * .8);
            bL.setPower(power * 1.2);
            bR.setPower(-power * 1.2);
        }
        else {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);
        }
    }

    public void moveRight(double power) {
        if (getAngle() > 1) {
            fL.setPower(power * .8);
            fR.setPower(-power * .8);
            bL.setPower(-power * 1.2);
            bR.setPower(power * 1.2);
        }
        else if (getAngle() < -1) {
            fL.setPower(power * 1.2);
            fR.setPower(-power * 1.2);
            bL.setPower(-power * .8);
            bR.setPower(power * .8);
        }
        else {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }



    public void approachStones (double power) {
        resetAngle();
        brakeMode();

        boolean cont = Double.isNaN(sensorDistanceLeft.getDistance(DistanceUnit.CM));


        while (cont == true && opmode.opModeIsActive()){
            moveLeft(power);

            if (sensorDistanceLeft.getDistance(DistanceUnit.CM) < 6) {
                cont = false;
            }





            opmode.telemetry.addData("front", sensorDistanceLeft.getDistance(DistanceUnit.CM));
            opmode.telemetry.addData("back", sensorDistanceBotBack.getDistance(DistanceUnit.CM));
            opmode.telemetry.update();
        }


        stopMotors();
        floatMode();
    }



    public void alignWithStones (double power) {

        brakeMode();
        while (opmode.opModeIsActive() && sensorColorLeft.red() > 300 || sensorColorBotBack.red() > 300) {

            if (sensorColorLeft.red() > 300) {

                moveBackward(power);
            }

            else if (sensorColorBotBack.red() > 300){
                //forward
                moveForward(power);
            }

            else {
                stopMotors();
            }




        }
        floatMode();
    }

    public void moveBackward (double power) {

        //reset();
        resetAngle();
        opmode.sleep(100);

        //brakeMode();
        runtime.reset();

        if (getAngle() > 1) {
            fL.setPower(1.2 * power);
            fR.setPower(0.8 * power);
            bL.setPower(1.2 * power);
            bR.setPower(0.8 * power);
        }
        else if (getAngle() < -1) {
            fL.setPower(.8 * power);
            fR.setPower(1.2 * power);
            bL.setPower(.8 * power);
            bR.setPower(1.2 * power);
        }
        else {
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }

        //stopMotors();

        //floatMode();
        //resetAngle();

    }

    public void moveForward(double power) {

        //reset();
        resetAngle();
        opmode.sleep(100);



        //brakeMode();
        runtime.reset();

        if (getAngle() > 1) {
            fL.setPower(.8 * power);
            fR.setPower(1.2 * power);
            bL.setPower(.8 * power);
            bR.setPower(1.2 * power);
        } else if (getAngle() < -1) {
            fL.setPower(1.2 * power);
            fR.setPower(.8 * power);
            bL.setPower(1.2 * power);
            bR.setPower(.8 * power);
        } else {
            fL.setPower(power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power);
        }

        //stopMotors();

       // floatMode();
        //resetAngle();
    }

}
