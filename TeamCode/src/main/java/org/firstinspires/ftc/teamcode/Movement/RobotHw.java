package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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
    private ElapsedTime runtime = new ElapsedTime();

    // Tick Conversion
    static final double COUNTS_PER_MOTOR_REV = 380;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

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

    public void goStraightGyro(double distance, double leftPower, double rightPower, double heading) {
        reset();
        // Forward
        if (distance > 0) {
            rightPower = rightPower - .15;
            leftPower = leftPower - .15;

        } // reverse
        else {
            rightPower = -rightPower + .15;
            leftPower = -leftPower + .15;
        }

        double target = Math.abs(distance * (537.6 / 15.5));

        brakeMode();

        if (distance > 0) {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {

                if (heading > 1) {
                    fL.setPower((.15 + (rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower((.15 + (rightPower * ((target - encoderAvg()) / target))));
                } else if (heading < -1) {
                    fL.setPower(.8 * (.15 + (rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(.8 * (.15 + (rightPower * ((target - encoderAvg()) / target))));
                } else {
                    fL.setPower((.15 + (rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower((.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
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
        } else {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
                if (heading > 1) {
                    fL.setPower((-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (-.15 + (-leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (-.15 + (-leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower((-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                } else if (heading < -1) {
                    fL.setPower(.8 * (-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower((-.15 + (-leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower((-.15 + (-leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(.8 * (-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                } else {
                    fL.setPower(-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target));
                    fR.setPower(-.15 + (-leftPower + .15) * ((target - encoderAvg()) / target));
                    bL.setPower(-.15 + (-leftPower + .15) * ((target - encoderAvg()) / target));
                    bR.setPower(-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target));
                }
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
        }

        stopMotors();

        floatMode();
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

}
