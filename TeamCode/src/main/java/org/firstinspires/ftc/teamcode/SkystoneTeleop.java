package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;


@TeleOp(name="Skystone TeleOP", group="Pushbot")

// @ AUTHOR ROBERT HAYDEN WARREN

public class SkystoneTeleop extends LinearOpMode{

    RobotHw robot = new RobotHw();

    public double rightstickx;
    public double leftstickx;
    public double leftstickyfront;
    public double leftstickyback;
    public int clampcount = 1;

    double fRtickspersecond  = 0.0;
    double fLtickspersecond  = 0.0;
    double bLtickspersecond  = 0.0;
    double bRtickspersecond  = 0.0;

    public double foundationDistance;
    public double liftHeight;
    public double liftDistance;
    public double liftAngle;

    public boolean teleop = true;

    public int constant = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this, teleop);

        ElapsedTime runTime = new ElapsedTime();
        //robot.brakeMode();
        runTime.reset();
        waitForStart();

        //robot.rotate.setPosition(.245);
        //robot.clamp.setPosition(.245);
        while (opModeIsActive()) {
            // Sin Cos Atan inputs for mecanum
            trigMecanum();

            fRtickspersecond = robot.fR.getCurrentPosition()/ runTime.seconds();
            fLtickspersecond = robot.fL.getCurrentPosition()/ runTime.seconds();
            bRtickspersecond = robot.bR.getCurrentPosition()/runTime.seconds();
            bLtickspersecond = robot.bL.getCurrentPosition()/ runTime.seconds();

            telemetry.addData("fRtickspersecond", fRtickspersecond);
            telemetry.addData("fLtickspersecond", fLtickspersecond);
            telemetry.addData("bRtickspersecond", bRtickspersecond);
            telemetry.addData("bLtickspersecond", bLtickspersecond);
            telemetry.update();

            // Foundation Clamp
            if (gamepad1.y) { //up
                robot.clamp.setPosition(0.2);
            }
            if (gamepad1.a) { //down
                robot.clamp.setPosition(1);
            }

            // Claw
            if (gamepad2.right_bumper) {
                robot.clawDown(); // down
            }
            if (gamepad2.left_bumper) {
                robot.clawUp(); // up
            }

            //Rotate
            if (gamepad2.left_trigger == 1){
                robot.rotate.setPosition(.6); // Cube fits in robot
            }
            if (gamepad2.right_trigger == 1){
                robot.rotate.setPosition(.245); // Cube is horizontal
            }
            // Red Grabber stone pick up
            if (gamepad1.x){
                robot.grabberRDown();
                robot.gripRUp();
            }

            // Red Grabber pinch and bring up stone
            if(gamepad1.b){
                robot.gripRDown();
                sleep(1000);
                robot.grabberRUp();
            }

            // Reverse Mode
            if (gamepad1.dpad_down){
                constant = -1; // Reverse
            }
            if (gamepad1.dpad_up){
                constant = 1; // Forward
            }

            //intake
            if (gamepad1.right_trigger > .1){
                robot.intakeL.setPower(gamepad1.right_trigger);
                robot.intakeR.setPower(-gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger > .1){
                robot.intakeL.setPower(-gamepad1.left_trigger);
                robot.intakeR.setPower(gamepad1.left_trigger);
            }
            else{
                robot.intakeL.setPower(0);
                robot.intakeR.setPower(0);
            }
            // Lift
            robot.liftExtend.setPower(gamepad2.right_stick_y);
            robot.liftRotate.setPower(-gamepad2.left_stick_y);
        }
    }

    public void trigMecanum() {

        rightstickx = -gamepad1.right_stick_x ;
        leftstickx = -gamepad1.left_stick_x * constant;

        leftstickyfront = gamepad1.left_stick_y * constant;
        leftstickyback = gamepad1.left_stick_y * -constant;

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
        robot.bL.setPower(v3);// * .79);
        robot.bR.setPower(v4);// * .79);
    }
}
