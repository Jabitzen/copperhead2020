package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Skystone TeleOP Blue", group="Pushbot")

// @ AUTHOR HAYDEN WARREN
public class SkystoneTeleopBlue extends LinearOpMode{

    RobotHw robot = new RobotHw();

    /*
    public DcMotor fL = null;
    public DcMotor fR = null;
    public DcMotor bL = null;
    public DcMotor bR = null;


    public Servo grabberL = null;
    public Servo grabberR = null;
    public Servo clamp = null;
*/
    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;

    public double foundationDistance;
    public double liftHeight = 0;
    public double liftDistance;
    public double liftAngle;

    public boolean automode = false;




    public int constant = 1;



    @Override
    public void runOpMode() throws InterruptedException {
/*
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
*/
        robot.init(this);
        waitForStart();

        while (opModeIsActive()) {
            // Sin Cos Atan inputs for mecanum
            trigMecanum();

            // Left grabber
            if (gamepad2.right_trigger > .1) {
                robot.grabberR.setPosition(.8); // Up
            }
            if (gamepad2.right_bumper) {
                robot.grabberR.setPosition(0.02); // Down
            }

            // Right grabber
            if (gamepad1.b) {
                robot.grabberL.setPosition(0.6); // Up
            }
            if (gamepad1.dpad_right) {
                robot.grabberL.setPosition(0.98); // Down
            }

            // Foundation Clamp
            if (gamepad1.a) {
                robot.clamp.setPosition(0); // Down
            }
            if (gamepad1.y) {
                robot.clamp.setPosition(.9); // Up
            }

            // Reverse Mode
            if (gamepad1.dpad_down){
                constant = -1; // Reverse
            }
            if (gamepad1.dpad_up){
                constant = 1; // Forward
            }


            robot.liftExtend.setPower(gamepad2.left_stick_y);
            robot.liftRotate.setPower(gamepad2.right_stick_y);

            if(gamepad2.x){
                automode = true;
                liftHeight = 0;
            }

            if(gamepad2.b){
                automode = false;
            }

            while(automode){
                //foundationDistance = sensor reading;
                if(gamepad2.dpad_up){
                    liftHeight+=12.7; //a stone is 12.7 cm high, in cm because thats the units of range sensor
                }

                if(gamepad2.dpad_up){
                    liftHeight-=12.7;
                }

                liftDistance = Math.sqrt((foundationDistance * foundationDistance) + (liftHeight * liftHeight));
                liftAngle = Math.asin(liftHeight/liftDistance);

                robot.rotateTo(liftAngle);
                robot.extendTo(liftDistance);


            }

            if(gamepad2.left_bumper){
                robot. intakeL.setPower(1);
            }
            else{
                robot.intakeL.setPower(0);
            }


            if(gamepad2.right_bumper){
                robot.intakeR.setPower(1);
            }
            else{
                robot.intakeR.setPower(0);
            }


            if(gamepad2.left_bumper){
                robot.liftExtend.setPower(-.5);
            }

            if(gamepad2.dpad_left){
                robot.liftRotate.setPower(.5);
            }

            if(gamepad2.dpad_right){
                robot.liftRotate.setPower(-.5);
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

        /*
        telemetry.addData("fl", v1);
        telemetry.addData ("fR", v2);
        telemetry.addData ("bL", v3);
        telemetry.addData ("bR", v4);
        telemetry.addData("leftX", gamepad1.left_stick_x);
        telemetry.addData("leftY", gamepad1.left_stick_y);
        telemetry.addData ("Right X", rightX);
        telemetry.update();

         */

        robot.fL.setPower(-v1);
        robot.fR.setPower(-v2);
        robot.bL.setPower(-v3);// * .79);
        robot.bR.setPower(v4);// * .79);
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
