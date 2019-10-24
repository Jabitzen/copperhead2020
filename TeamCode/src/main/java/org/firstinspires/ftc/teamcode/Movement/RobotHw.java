package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo grabber = null;
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
        grabber = hwMap.get(Servo.class, "grabber");



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

        while (encoderAvg() < (distance * (537.6/18.5))) {
            fL.setPower(-rightPower);
            fR.setPower(leftPower);
            bL.setPower(leftPower);
            bR.setPower(rightPower);
            opmode.telemetry.addData("avg", encoderAvg());
            opmode.telemetry.addData("fl", fL.getCurrentPosition());
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.update();
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }

    public void correctStraight (double distance, double power) {
        reset();
        while (encoderAvg() < (distance * (537.6/18.5))) {
            fL.setPower(-power +0.1);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(power - 0.1);
        }
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }






    public void goInches(double distance, double power) {
        dtEncoderModeOn();
        targetPosition(distance);
        setMode();
        runtime.reset();
        fL.setPower(power);
        fR.setPower(power);
        bL.setPower(power);
        bR.setPower(power);

        while (opmode.opModeIsActive() && runtime.seconds() < 10.0 && fL.isBusy() && fR.isBusy()  && bR.isBusy()) {
            //bL.setPower(power);
            opmode.telemetry.addData("fl", fL.getCurrentPosition()) ;
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.update();
        }

        stopMotors();

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        //reset();
        dtEncoderModeOn();
        targetPositionStrafeRight(distance);
        setMode();
        runtime.reset();
        fL.setPower(power);
        fR.setPower(power);

        bR.setPower(power);

        while (opmode.opModeIsActive() && runtime.seconds() < 10.0 && fL.isBusy() && fR.isBusy()  && bR.isBusy()) {
            //bL.setPower(power);
            opmode.telemetry.addData("fl", fL.getCurrentPosition()) ;
            opmode.telemetry.addData("fr", fR.getCurrentPosition());
            opmode.telemetry.addData("bl", bL.getCurrentPosition());
            opmode.telemetry.addData("br", bR.getCurrentPosition());
            opmode.telemetry.update();
        }

        stopMotors();

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    /*public void strafeLeft (double distance, double power) {
        reset();
        dtEncoderModeOn();
        while (Math.abs(encoderAvg()) < Math.abs(distance * COUNTS_PER_INCH)) {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }*/

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

    public void grabberDown(){
        grabber.setPosition(1);
    }

    public void grabberUp(){
        grabber.setPosition(.6);
    }




}
