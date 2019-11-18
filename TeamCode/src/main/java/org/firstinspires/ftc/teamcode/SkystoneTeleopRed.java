package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Skystone TeleOP Red", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class SkystoneTeleopRed extends LinearOpMode{


    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;

    public Servo grabberL = null;
    public Servo grabberR = null;
    public Servo clamp = null;

    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;

    public int constant;


    @Override
    public void runOpMode() throws InterruptedException {

        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR  = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR  = hardwareMap.get(DcMotor.class, "bR");

        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");
        clamp = hardwareMap.get(Servo.class, "clamp");

        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            // Sin Cos Atan inputs for mecanum
            trigMecanum();

            // Left grabber
            if (gamepad2.right_trigger > .1) {
                grabberL.setPosition(0.3); // Up
            }
            if (gamepad2.right_bumper) {
                grabberL.setPosition(0.98); // Down
            }

            // Right grabber
            if (gamepad1.b) {
                grabberR.setPosition(.4); // Up
            }
            if (gamepad1.dpad_right) {
                grabberR.setPosition(0.02); // Down
            }

            // Foundation Clamp
            if (gamepad1.a) {
                clamp.setPosition(0); // Down
            }
            if (gamepad1.y) {
                clamp.setPosition(.9); // Up
            }

            // Reverse Mode
            if (gamepad1.dpad_down){
                constant = -1; // Reverse
            }
            if (gamepad1.dpad_up){
                constant = 1; // Forward
            }
        }
    }

    public void trigMecanum() {
        rightstickx = gamepad1.right_stick_x ;
        leftstickx = gamepad1.left_stick_x * constant;

        leftstickyfront = gamepad1.left_stick_y * -constant;
        leftstickyback = gamepad1.left_stick_y * constant;

        double rFront = Math.hypot(rightstickx, leftstickyfront);
        double rBack = Math.hypot(rightstickx, leftstickyback);

        double robotAngleFront = Math.atan2(leftstickyfront, rightstickx) - Math.PI / 4;
        double robotAngleBack = Math.atan2(leftstickyback, rightstickx) - Math.PI / 4;

        double rightX = leftstickx;

        final double v1 = rFront * Math.cos(robotAngleFront) + rightX;
        final double v2 = rFront * Math.sin(robotAngleFront) - rightX;
        final double v3 = rBack * Math.sin(robotAngleBack) + rightX;
        final double v4 = rBack * Math.cos(robotAngleBack) - rightX;

        //telemetry.addData ("r", r);
        //telemetry.addData ("robotAngle", robotAngle);
        //telemetry.update();

        //telemetry.addData("input",gamepad1.left_stick_y *gamepad1.left_stick_y);
        //telemetry.addData("fl", v1);
        //telemetry.addData ("fR", v2);
        //telemetry.addData ("bL", v3);
        //telemetry.addData ("bR", v4);
        //telemetry.addData ("Right X", rightX);
        //telemetry.update();

        fL.setPower(-v1);
        fR.setPower(-v2);
        bL.setPower(-v3);// * .79);
        bR.setPower(v4);// * .79);
    }
    /* -----------------------------Deadzone Calculations--------------------------------------
    public double rightStickX() {
        if (gamepad1.right_stick_x > 0.15){
            //rightstickx = ((gamepad1.right_stick_x * gamepad1.right_stick_x) + .39);

            rightstickx = Range.scale((gamepad1.right_stick_x * gamepad1.right_stick_x),.15, 1, 0, 1);

        }

        else if(gamepad1.right_stick_x < -0.15){
            //rightstickx = (-1 * ((gamepad1.right_stick_x * gamepad1.right_stick_x) + .39));

            rightstickx = Range.scale((-1 * gamepad1.right_stick_x * gamepad1.right_stick_x),-.15, -1, 0, -1);
        }

        else{
            rightstickx = 0;
        }

        return rightstickx;
    }

    public double leftStickY() {
        if (gamepad1.left_stick_y > 0.3){
            //leftsticky = ((gamepad1.left_stick_y * gamepad1.left_stick_y) + .39) * constant;

            leftsticky = Range.scale((gamepad1.left_stick_y * gamepad1.left_stick_y * constant),.3, 1, 0, 1);
        }

        else if(gamepad1.left_stick_y < -0.3){
            //leftsticky = (-1 * ((gamepad1.left_stick_y * gamepad1.left_stick_y) + .39)) * constant;

            leftsticky = Range.scale((-1 * gamepad1.left_stick_y * gamepad1.left_stick_y * constant),-.3, -1, 0, -1);
        }

        else{
            leftsticky = 0;
        }

        return leftsticky;
    }
    public double leftStickX() {
        if (gamepad1.left_stick_x > 0.15){
            //leftstickx = ((gamepad1.left_stick_x * gamepad1.left_stick_x) + .39) * constant;

            leftstickx = Range.scale((gamepad1.left_stick_x * gamepad1.left_stick_x * constant),.15, 1 , 0, 1);
        }

        else if(gamepad1.left_stick_x < -0.15){
            //leftstickx = (-1 * ((gamepad1.left_stick_x * gamepad1.left_stick_x) + .39)) * constant;

            leftstickx = leftstickx = Range.scale((-1 * gamepad1.left_stick_x * gamepad1.left_stick_x * constant),-.15, -1, 0, -1);
        }

        else{
            leftstickx = 0;
        }

        return leftstickx;
    }
    */

}
