package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHw {

    // Motors
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;

    DcMotor intakeR = null;
    DcMotor intakeL = null;

    // Servos

    Servo clip = null;
    Servo claw = null;
    Servo rotate = null;
    public Servo clamp = null;
    public Servo grabberL = null;
    public Servo grabberR = null;

    //Servo grabber = null;

    // HardwareMap
    HardwareMap hwMap;

    // Linear Opmode
    LinearOpMode opmode;

    // Time
    private ElapsedTime runtime = new ElapsedTime();

    // Tick Conversion
    static final double     COUNTS_PER_MOTOR_REV    = 380 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);





    public void init(LinearOpMode lOpmode) {
        opmode = lOpmode;
        // Hardware map
        hwMap = opmode.hardwareMap;

        // Define and Initialize Motors
        fL  = opmode.hardwareMap.get(DcMotor.class, "fL");
        fR  = opmode.hardwareMap.get(DcMotor.class, "fR");
        bL  = opmode.hardwareMap.get(DcMotor.class, "bL");
        bR  = opmode.hardwareMap.get(DcMotor.class, "bR");

        //intakeR  = hwMap.get(DcMotor.class, "intakeR");
        //intakeL  = hwMap.get(DcMotor.class, "intakeL");

        //Define and initialize servos
        clip  = hwMap.get(Servo.class, "clip");
        claw  = hwMap.get(Servo.class, "claw");
        rotate  = hwMap.get(Servo.class, "rotate");
        clamp = hwMap.get(Servo.class, "clamp");
        grabberL = hwMap.get(Servo.class, "grabberL");
        grabberR = hwMap.get(Servo.class, "grabberR");



        // grabber = hwMap.get(Servo.class, "grabber");

        //set direction of motors
        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        dtEncoderModeOn();

        //intakeL.setDirection(DcMotor.Direction.FORWARD);
        //intakeR.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        opmode.telemetry.addData("fl", fL.getCurrentPosition()) ;
        opmode.telemetry.addData("fr", fR.getCurrentPosition());
        //opmode.telemetry.addData("bl", bL.getCurrentPosition());
        opmode.telemetry.addData("br", bR.getCurrentPosition());
        opmode.telemetry.update();



        //intakeL.setPower(0);
        //intakeR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double atTarget(double distance){
        return Math.abs(distance*COUNTS_PER_INCH);
    }

    public void moveStraight (double distance, double power) {
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
        }else if (bR.getCurrentPosition() != 0) {
            while (Math.abs(bR.getCurrentPosition() - startVal) < atTarget(distance)) {
                fL.setPower(power);
                fR.setPower(power);
                bL.setPower(power);
                bR.setPower(power);
            }
        }
    }

    public void goStraight(double distance, double leftPower, double rightPower) {
        reset();
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

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (distance > 0) {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
                fL.setPower(-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target));
                fR.setPower(.15 + (leftPower * ((target - encoderAvg()) / target)));
                bL.setPower(.15 + (leftPower * ((target - encoderAvg()) / target)));
                bR.setPower(.15 + (rightPower * ((target - encoderAvg()) / target)));
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
        }
        else {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
                fL.setPower(.15 + (-rightPower - .15) * ((target - encoderAvg()) / target));
                fR.setPower(-.15 + (leftPower * ((target - encoderAvg()) / target)));
                bL.setPower(-.15 + (leftPower * ((target - encoderAvg()) / target)));
                bR.setPower(-.15 + (rightPower * ((target - encoderAvg()) / target)));
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
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void goStraightGyro(double distance, double leftPower, double rightPower, double heading) {
        reset();
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

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (distance > 0) {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
                heading = heading;
                if (heading > 1) {
                    fL.setPower((-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower((.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else if (heading < -1) {
                    fL.setPower(.8 * (-.15 + (-rightPower + .15) * ((target - encoderAvg()) / target)));
                    fR.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower((.15 + (leftPower * ((target - encoderAvg()) / target))));
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
                opmode.telemetry.addData("angle", heading);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        }
        else {
            while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
                heading = heading;
                if (heading > 1) {
                    fL.setPower((.15 + (-rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower(.8 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower(.8 * (-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower((-.15 + (rightPower * ((target - encoderAvg()) / target))));
                }
                else if (heading < -1) {
                    fL.setPower(.8 * (.15 + (-rightPower - .15) * ((target - encoderAvg()) / target)));
                    fR.setPower((-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bL.setPower((-.15 + (leftPower * ((target - encoderAvg()) / target))));
                    bR.setPower(.8 * (-.15 + (rightPower * ((target - encoderAvg()) / target))));
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
                opmode.telemetry.addData("angle", heading);
                opmode.telemetry.addData("fl", fL.getPower());
                opmode.telemetry.addData("fr", fR.getPower());
                opmode.telemetry.addData("bl", bL.getPower());
                opmode.telemetry.addData("br", bR.getPower());
                opmode.telemetry.update();
            }
        }

        stopMotors();

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void stopMotors() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
    public void targetPosition(double inches) {
        reset();

        fL.setTargetPosition(fL.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
        //bL.setTargetPosition(bL.getCurrentPosition() + int)(inches * COUNTS_PER_INCH));
        bR.setTargetPosition(bR.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
        fR.setTargetPosition(fR.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
    }

    public void targetPositionStrafeRight(double inches) {
        fL.setTargetPosition(fL.getCurrentPosition() + 4500);//(int)(inches * COUNTS_PER_INCH));
        //bL.setTargetPosition(bL.getCurrentPosition() + 4500);//(int)(inches * COUNTS_PER_INCH));
        bR.setTargetPosition(bR.getCurrentPosition() - 4500);//(int);(inches * COUNTS_PER_INCH));
        fR.setTargetPosition(fR.getCurrentPosition() - 4500);//(int)(inches * COUNTS_PER_INCH));
    }

    public void setMode() {
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void strafeRight (double distance, double power) {
        reset();
        double target = Math.abs(distance * (537.6/11));

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
            fL.setPower(power);
            fR.setPower(power * 1.2);
            bL.setPower(-power);
            bR.setPower(power);
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


        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void strafeLeft (double distance, double power) {
        reset();


        double target = Math.abs(distance * (537.6/11));

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(encoderAvg()) < target && opmode.opModeIsActive()) {
            fL.setPower(-power);
            fR.setPower(-power);
            bL.setPower(power);
            bR.setPower(-power);
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


        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }





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

    public double encoderAvg() {
        double avg = 0;

        // FR motor
        avg += Math.abs(fR.getCurrentPosition());

        // FL motor
        avg += Math.abs(fL.getCurrentPosition());

        //BR motor
        avg += Math.abs(bR.getCurrentPosition());



        return avg/3;

    }


    public void dtEncoderModeOn (){
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnTime (double power, boolean turnRight, double time){
        ElapsedTime timer = new ElapsedTime();
        if (turnRight) {
            while (timer.seconds() < time) {
                fL.setPower(power);
                bL.setPower(power);
                fR.setPower(-power);
                bR.setPower(-power);
            }
        }
        else{
            while (timer.seconds() < time) {
                fL.setPower(-power);
                bL.setPower(-power);
                fR.setPower(power);
                bR.setPower(power);
            }
        }
    }
}
