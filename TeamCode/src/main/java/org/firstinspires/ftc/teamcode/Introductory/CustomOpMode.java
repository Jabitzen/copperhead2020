package org.firstinspires.ftc.teamcode.Introductory;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class CustomOpMode {

    public DcMotor leftDrive;
    ElapsedTime timer = new ElapsedTime();
    public DcMotor rightDrive;
    public Servo dropJewel;

    public BNO055IMU gyro;

    Orientation angles;
    Acceleration gravity;


    HardwareMap hwMap;
   // HardwareMap hwMap = null;

    public CustomOpMode(LinearOpMode className) {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

       gyro = className.hardwareMap.get(BNO055IMU.class, "imu");
       gyro.initialize(parameters);


       leftDrive = className.hardwareMap.dcMotor.get("left_drive");
       rightDrive = className.hardwareMap.dcMotor.get("right_drive");
       dropJewel = className.hardwareMap.servo.get("drop_jewel");

        //intake= hwMap.get(DcMotor.class, "intake");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //intake.setPower(0);


        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //block = hwMap.get(Servo.class, "block");
    }

    public void runOpMode() throws InterruptedException {
    }

    public double getGyroYaw() {
        double yaw = angles.firstAngle * -1;         //if turn no work change this
        return yaw;
    }

    public void jewel() {
        dropJewel.setPosition(0.8);

    }

    public void moveStraight (double power, double distance) {
        int startVal;
        startVal = leftDrive.getCurrentPosition();
        while (Math.abs(leftDrive.getCurrentPosition() - startVal) < distance) {
            leftDrive.setPower(power);
            rightDrive.setPower(power);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void turn (double direction, double power, double distance) {
        double currentHeading = getGyroYaw();
        while (Math.abs(distance) > currentHeading) {
            leftDrive.setPower(power * direction);
            rightDrive.setPower(-power*direction);

        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turn_left (double power, double distance) {
        int startVal = leftDrive.getCurrentPosition();
        while ((leftDrive.getCurrentPosition() - startVal) < distance) {
            leftDrive.setPower(-power);
            rightDrive.setPower(power);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turn_right (double power, double distance) {
        int startVal = rightDrive.getCurrentPosition();
        while ((rightDrive.getCurrentPosition() - startVal) < distance) {
            leftDrive.setPower(power);
            rightDrive.setPower(-power);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
