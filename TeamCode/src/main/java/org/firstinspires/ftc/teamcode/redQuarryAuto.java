package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Movement.RobotHw;
import org.firstinspires.ftc.teamcode.Vision.BitMapVision;

@Autonomous(name="redQuarryAuto", group="12596")
public class redQuarryAuto extends LinearOpMode {

    BitMapVision bm1 = null;
    RobotHw robot = new RobotHw(this);



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //bm1 = new BitMapVision(this);
        waitForStart();

        // Middle Pathing
        robot.goStraight(); // Move to Skystone
        sleep(3000);
        telemetry.addLine("done");
        telemetry.update();

        //robot.grabberDown(); // Bring down grabber
      //  robot.strafeLeft(10, .5);
        /*
        String pos = bm1.findSkystones();
        sleep(3000);
        telemetry.addData("Skystone Position: ", pos);
        telemetry.update();
        sleep(5000);


        if (pos == "left") {
            robot.moveStraight(30, 0.5); //move to skystone
            robot.grabberDown();
            robot.moveStraight(5, .5);
            robot.strafeLeft(60, 0.5);
            robot.grabberUp();
            robot.strafeRight(85, .5);
            robot.grabberDown();
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
        */

    }
}
