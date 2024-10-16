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
            // editing
//            robot.pixGrab();
            //robot.turnSlideUp();
            robot.wrist_mid();
            robot.moveBackwardToPosition(robot_power, 25, 2);
//            robot.turnSlide(robot_power, 32, 2, FALSE);
            robot.expandSlideForLatching();
            robot.wrist_drop();
            sleep(100);
            robot.moveForwardToPosition(robot_power, 2, 1);
            robot.wrist_grab();
            sleep(100);
            robot.moveForwardToPosition(robot_power, 5, 1);
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
