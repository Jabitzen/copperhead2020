package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;

@Autonomous(name="forward 5 in", group="12596")
public class goForward extends LinearOpMode {

    RobotHw robot = new RobotHw();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this);
        waitForStart();



        //robot.turnPID( 90, .43/90, 0, 0, 7);
        //robot.grabberRDown();
        robot.goStraightGyro(20, .5, 4);
        sleep(1000);
        //robot.approachStonesRed(.5);
        //sleep(1000);
        //robot.rotate(90, .3);
        //sleep(1000);
        //robot.strafeRightGyro(40, .4);
        //sleep(1000);
        //robot.fL.setPower(1);
        //robot.approachStonesRed(.5);
        /*robot.fL.setPower(0.5);
        robot.fR.setPower(-0.5);
        robot.bL.setPower(0.5);
        robot.bR.setPower(-0.5);*/
       // robot.alignStonesB(0.2);
        //robot.goStraightGyro(7, 0.5, 5);
        sleep(5000);
        }
}