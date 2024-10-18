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

            robot.hangElementOnHighBar(robot.robot_power);
//
//            int turn_deg = (int) robot.getAngle();
////            robot.rotateAntiClock(turn_deg, robot_power);
//            telemetry.addData("Current agnel is", "%5d", turn_deg);
//            telemetry.update();
//            robot.turnSlideBack();

            // go to first element
            robot.rotateAntiClock(-90, robot.robot_power);
            robot.moveForwardToPosition(robot.robot_power, 46, 3000);
            robot.rotateAntiClock(-90, robot.robot_power);

            // pick up and drop pixel
            robot.turnSlideBack();
//            robot.moveForwardToPosition(robot_power, 1, 1);
            robot.pixGrab();
            robot.turnSlideUp();
            robot.rotateAntiClock(-37, robot.robot_power);
            robot.moveBackwardToPosition(robot.robot_power, 20, 2400);
            robot.moveRightToPosition(robot.robot_power, 4, 1500);
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
