package org.firstinspires.ftc.teamcode.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHw {

    // Motors
    DcMotor fL = null;
    DcMotor fR = null;
    DcMotor bL = null;
    DcMotor bR = null;

    DcMotor intakeR = null;
    DcMotor intakeL = null;

    // Servos
    Servo clip = null;
    Servo claw = null;
    Servo rotate = null;

    // HardwareMap
    HardwareMap hwMap;

    // Time
    private ElapsedTime runtime = new ElapsedTime();

    // Tick Conversion
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public void init(HardwareMap ahwMap) {
        // Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        fL  = hwMap.get(DcMotor.class, "fL");
        fR  = hwMap.get(DcMotor.class, "fR");
        bL  = hwMap.get(DcMotor.class, "bL");
        bR  = hwMap.get(DcMotor.class, "bR");

        intakeR  = hwMap.get(DcMotor.class, "intakeR");
        intakeL  = hwMap.get(DcMotor.class, "intakeL");

        //Define and initialize servos
        clip  = hwMap.get(Servo.class, "clip");
        claw  = hwMap.get(Servo.class, "claw");
        rotate  = hwMap.get(Servo.class, "rotate");

        //set direction of motors
        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);

        intakeL.setDirection(DcMotor.Direction.FORWARD);
        intakeR.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        fL.setPower(0);
        bL.setPower(0);
        fR.setPower(0);
        bR.setPower(0);
        intakeL.setPower(0);
        intakeR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double atTarget(double distance){
        return Math.abs(distance*COUNTS_PER_INCH);
    }

    public void moveStraight (double distance, int power) {
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

    public void strafeLeft (double distance, int power) {
        dtEncoderModeOn();
        int startVal = fL.getCurrentPosition();
        while (Math.abs(fL.getCurrentPosition() - startVal) < Math.abs(distance * COUNTS_PER_INCH)) {
            fL.setPower(power);
            fR.setPower(-power);
            bL.setPower(-power);
            bR.setPower(power);
        }
    }

    public void strafeRight (double distance, int power) {
        dtEncoderModeOn();
        int startVal = fR.getCurrentPosition();
        while (Math.abs(fR.getCurrentPosition() - startVal) < Math.abs(distance * COUNTS_PER_INCH)) {
            fL.setPower(-power);
            fR.setPower(power);
            bL.setPower(power);
            bR.setPower(-power);
        }
    }





    public void dtEncoderModeOn (){
        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }




}
