package org.firstinspires.ftc.teamcode;
import static java.lang.Boolean.FALSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="BLUE Basket", group = "Autonomous")
//@Disabled
public class TeamAutoBlueBasket extends LinearOpMode {
    private final double robot_power = 1.0;
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
            //robot.wrist_mid();
            robot.moveBackwardToPosition(robot_power, 33, 2400);
            //robot.wrist_drop();
            robot.expandSlideForLatching();
            robot.wrist_drop();
//            robot.moveBackwardToPosition(robot_power, 3, 500);
            sleep(400);
            robot.moveForwardToPosition(robot_power, 2, 1);
            robot.wrist_grab();
            sleep(100);
            robot.moveForwardToPosition(robot_power, 5, 1);
            robot.pixRelease();
            robot.contractSlideAfterLatching();

            int turn_deg = (int) robot.getAngle();
//            robot.rotateAntiClock(turn_deg, robot_power);
            telemetry.addData("Current agnel is", "%5d", turn_deg);
            telemetry.update();
//            robot.turnSlideBack();
            robot.rotateAntiClock(-90, robot_power);
            robot.moveForwardToPosition(robot_power, 46, 3);
            robot.rotateAntiClock(-90, robot_power);
            robot.turnSlideBack();
//            robot.moveForwardToPosition(robot_power, 1, 1);
            robot.pixGrab();
            robot.turnSlideUp();
            robot.rotateAntiClock(-37, robot_power);
            robot.moveBackwardToPosition(robot_power, 20, 2400);
            robot.moveRightToPosition(robot_power, 4, 3);
            robot.expandSlide();
            robot.pixRelease();

//            sleep(500);
//            robot.moveBackwardToPosition(robot_power, 2, 1);
//            robot.pixRelease();
//            robot.contractSlideAfterLatching();
//            robot.turnSlide(robot_power, 0, 2, FALSE);
            break;
        }
    }

}
