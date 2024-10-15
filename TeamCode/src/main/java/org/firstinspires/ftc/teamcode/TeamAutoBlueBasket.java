package org.firstinspires.ftc.teamcode;
import static java.lang.Boolean.FALSE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Autonomous(name="BLUE Basket", group = "Autonomous")
//@Disabled
public class TeamAutoBlueBasket extends LinearOpMode {
    private final double robot_power = 1.0;
    BBRobot robot;

    @Override
    public void runOpMode()
    {
        robot = new BBRobot(hardwareMap, telemetry);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // editing
            robot.pixGrab();
            //robot.turnSlideUp();
            //robot.moveForwardToPosition(robot_power, 12, 2);
            robot.turnSlide(robot_power, 32, 2, FALSE);
            robot.expandSlideForLatching();
            robot.wrist_grab();
            sleep(500);
            robot.pixRelease();
            robot.moveBackwardToPosition(robot_power, 2, 1);
            robot.contractSlideAfterLatching();
            robot.turnSlide(robot_power, 0, 2, FALSE);
            break;
        }
    }

}
