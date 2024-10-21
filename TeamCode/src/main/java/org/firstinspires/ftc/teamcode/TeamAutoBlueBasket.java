package org.firstinspires.ftc.teamcode;
import static java.lang.Boolean.FALSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="BLUE Basket", group = "Autonomous")
//@Disabled
public class TeamAutoBlueBasket extends LinearOpMode {
    //private final double robot_power = 1.0;
    BBRobot robot;

//    TeamAutoBlueBasket(){
//        robot = new BBRobot(hardwareMap, telemetry);
//        initRobotSettings();
//    }

    private void initRobotSettings() {
        robot.pixGrab();
//        robot.turnSlideUp();
    }

    @Override
    public void runOpMode()
    {
//        new TeamAutoBlueBasket();
        robot = new BBRobot(hardwareMap, telemetry);
        initRobotSettings();
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //move towards the center
            robot.moveBackwardToPosition(robot.robot_power, 30, 2400);

            //hang element on the bard
            robot.hangElementOnHighBar(robot.robot_power);
//
//            int turn_deg = (int) robot.getAngle();
////            robot.rotateAntiClock(turn_deg, robot_power);
//            telemetry.addData("Current agnel is", "%5d", turn_deg);
//            telemetry.update();

            // go to first element
            robot.rotateAntiClock(-90, robot.robot_power);
            robot.moveForwardToPosition(robot.robot_power, 50, 4500);
            robot.rotateAntiClock(-90, robot.robot_power);

            // pick up and drop pixel7
            robot.turnSlideBack();
            robot.moveForwardToPosition(robot.robot_power, 3, 1);
            robot.pixGrab();
            robot.turnSlideForDrop();
            robot.rotateAntiClock(-45, robot.robot_power);
            robot.moveRightToPosition(robot.robot_power, 14, 1500);
            robot.moveBackwardToPosition(robot.robot_power, 17,  2400);
            robot.expandSlide();
            robot.wrist_end();
            sleep(500);
            robot.pixRelease();

            sleep(500);
            robot.contractSlide();

            // reset the slide to original position
            robot.turnSlideUp();
            break;
        }
    }

}
