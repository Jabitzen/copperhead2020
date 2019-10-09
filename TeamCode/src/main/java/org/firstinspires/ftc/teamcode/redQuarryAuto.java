package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;

@Autonomous(name="redQuarryAuto", group="12596")
public class redQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;
    RobotHw robot = new RobotHw();



    @Override
    public void runOpMode() throws InterruptedException {
        bm1 = new BitMapVision(this);
        waitForStart();

        String pos = bm1.findSkystones();
        sleep(3000);
        telemetry.addData("Skystone Position: ", pos);
        telemetry.update();
        sleep(5000);

        if (pos == "left") {
            robot.moveStraight(30, 0.5); //move to skystone
            //grabber down
            robot.moveStraight(5, .5);
            robot.strafeLeft(60, 0.5);
            //grabber up
            robot.strafeRight(85, .5);
            //grabber down
            robot.strafeLeft(85, 0.5);
            robot.strafeRight(20, .5);

        }

        else if (pos == "middle") {

        }

        else if (pos == "right"){

        }

        else {
            telemetry.addLine("oopsie");
        }
    }
}
