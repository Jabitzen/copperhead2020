package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Testing", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class testing extends LinearOpMode{


    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;

    public Servo grabber = null;
    public Servo clamp = null;

    @Override
    public void runOpMode() throws InterruptedException {
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR  = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR  = hardwareMap.get(DcMotor.class, "bR");

        grabber = hardwareMap.get(Servo.class, "grabber");
        clamp = hardwareMap.get(Servo.class, "clamp");

        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();
        while (opModeIsActive()) {
            //mecanumDrive_Cartesian(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            trigMecanum();

            if (gamepad1.b) {
                grabber.setPosition(0.6);
            }

            if (gamepad1.a) {
                grabber.setPosition(1);
            }

            if (gamepad1.x) {
                clamp.setPosition(0);
            }

            if (gamepad1.y) {
                clamp.setPosition(1);
            }

            telemetry.addData ("pos1", clamp.getPosition());
       //     telemetry.addData ("pos2", clamp.getPosition());
            telemetry.update();
        }




    }
    public void mecanumDrive_Cartesian(double x, double y, double rotation)
    {
        double wheelSpeeds[] = new double[4];

        wheelSpeeds[0] = x + y + rotation;
        wheelSpeeds[1] = -x + y - rotation;
        wheelSpeeds[2] = -x + y + rotation;
        wheelSpeeds[3] = x + y - rotation;

        normalize(wheelSpeeds);

        fL.setPower(-wheelSpeeds[0]);
        fR.setPower(wheelSpeeds[1]);
        bL.setPower(wheelSpeeds[2]);
        bR.setPower(wheelSpeeds[3]);
    }   //mecanumDrive_Cartesian

    public void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);

        for (int i = 1; i < wheelSpeeds.length; i++)
        {
            double magnitude = Math.abs(wheelSpeeds[i]);

            if (magnitude > maxMagnitude)
            {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0)
        {
            for (int i = 0; i < wheelSpeeds.length; i++)
            {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }

    public void trigMecanum() {
        double rightstickx = 0;
        double leftstickx = 0;
        double leftsticky = 0;



        if (gamepad1.right_stick_x > 0.15){
            rightstickx = (gamepad1.right_stick_x * gamepad1.right_stick_x) + .3;
        }

        else if(gamepad1.right_stick_x < -0.15){
            rightstickx = -1 * ((gamepad1.right_stick_x * gamepad1.right_stick_x) + .3);
        }

        else{
            rightstickx = 0;
        }



        if (gamepad1.left_stick_x > 0.15){
            leftstickx = (gamepad1.left_stick_x * gamepad1.left_stick_x) + .3;
        }

        else if(gamepad1.left_stick_x < -0.15){
            leftstickx = -1 * ((gamepad1.left_stick_x * gamepad1.left_stick_x) + .3);
        }

        else{
            leftstickx = 0;
        }



        if (gamepad1.left_stick_y > 0.3){
            leftsticky = (gamepad1.left_stick_y * gamepad1.left_stick_y) + .3;
        }

        else if(gamepad1.left_stick_y < -0.3){
            leftsticky = -1 * ((gamepad1.left_stick_y * gamepad1.left_stick_y) + .3);
        }

        else{
            leftsticky = 0;
        }

        telemetry.addLine("start");
        double r = Math.hypot(rightstickx, leftsticky);
        double robotAngle = Math.atan2(leftsticky, rightstickx) - Math.PI / 4;
        double rightX = leftstickx;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        //telemetry.addData ("r", r);
        //telemetry.addData ("robotAngle", robotAngle);
       // telemetry.update();

        telemetry.addData("fl", v1);
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
       // telemetry.addData ("Right X", rightX);
        telemetry.update();

        fL.setPower(v1);
        fR.setPower(v2);
        bL.setPower(v3 * .85);
        bR.setPower(v4 * .85);
    }


}
