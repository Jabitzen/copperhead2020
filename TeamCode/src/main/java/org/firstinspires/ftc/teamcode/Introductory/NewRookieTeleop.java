package org.firstinspires.ftc.teamcode.Introductory;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="NewRookieTeleop", group="Pushbot")
//@Disabled

public class NewRookieTeleop extends OpMode {

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor intakeOutake;
    public Servo dropWall;
    public Servo dropJewel;

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */


        leftDrive = hardwareMap.dcMotor.get("left_drive");
        //hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");
        intakeOutake = hardwareMap.dcMotor.get("intake_outake");
        dropWall = hardwareMap.servo.get("drop_wall");
        dropJewel = hardwareMap.servo.get("drop_jewel");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intakeOutake.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // robot.block.setPosition(0);

    }

    public void loop() {
        double leftStick = gamepad1.left_stick_y;
        double rightStick = gamepad1.right_stick_x;
        //boolean intake;
        // boolean block;

        double leftPower = leftStick + rightStick;
        double rightPower = leftStick - rightStick;

        //right power = 1.5 / 1.5 >>>> 1
        //left power = 1 / 1.5 >>>> .6666
        double max;
        max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (max > 1) {
            leftPower /= max;
            rightPower /= max;
        }

        if ((Math.abs(leftPower) > 0.05) || (Math.abs(rightPower) > 0.05)) {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        } else {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


        if (gamepad1.left_trigger > 0.1) {
            telemetry.addLine("running");
            telemetry.update();
            intakeOutake.setPower(gamepad1.left_trigger);

        }

        else if (gamepad1.left_trigger < 0.1) {
            intakeOutake.setPower(0);
        }

        if (gamepad1.right_trigger > 0.1) {

            telemetry.update();
            intakeOutake.setPower(-gamepad1.right_trigger);
        }

        else if (gamepad1.right_trigger < 0.1) {
            intakeOutake.setPower(0);
        }


        boolean berlinWallUp = gamepad1.left_bumper;
        boolean berlinWalldown = gamepad1.right_bumper; //need one for reset at beginning?

        if (gamepad1.left_bumper) {

            dropWall.setPosition(1);
        }

        if (gamepad1.right_bumper) {

            dropWall.setPosition(0);
        }

       /* if (gamepad2.left_bumper) {

            telemetry.addLine("running");
            dropJewel.setPosition(1);


        }

        if (gamepad2.right_bumper) {

            telemetry.addLine("running");

            dropJewel.setPosition(0);
        } */




        /*boolean test_jewel_servo = gamepad1.left_bumper;

        if (gamepad1.left_bumper) {
            dropJewel.setPosition(0);
        }

        if (gamepad1.right_bumper) {
            dropJewel.setPosition(1);
        } */


    }
}

