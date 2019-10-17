package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;

@Autonomous(name="redFoundationAuto", group="12596")
public class redFoundationAuto extends LinearOpMode {
    RobotHw robot = new RobotHw();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        robot.moveStraight(-15, .5);
        robot.turnTime(.5, true, .3);
        robot.moveStraight(-15, .5);
        //servo down
        robot.turnTime(.5, false, .3);
        robot.moveStraight(15, .5);
        //servo up
        robot.strafeRight(30, .5);
    }
}
