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


        //robot.goStraightGyro(70, .3, 7);
        //sleep(1000);
        //robot.rotate(90, .3);
        //sleep(1000);
        //robot.strafeRightGyro(40, .4);
        //sleep(1000);
        robot.goStraightGyro(5, .4, 3);
        sleep(30000);
        }
}