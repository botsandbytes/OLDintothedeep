package org.firstinspires.ftc.teamcode;
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

        while (opModeIsActive())
        {

        }
    }

}
