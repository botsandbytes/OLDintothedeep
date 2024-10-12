package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopTest extends LinearOpMode {
    private final double robot_power = 1.0;

    @Override
    public void runOpMode()  {

        BBRobot robot = new BBRobot(hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData("Status", "Initialized"); //Displays "Status: Initialized"
        telemetry.update();
        //Driver must press INIT and then ▶️

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("TeleOP", "New Code");
            telemetry.update();

            /********** GamePad 1 ****************/
            //Turning
            if (this.gamepad1.left_stick_x < 0.5 && this.gamepad1.left_stick_x > -0.5) {
                robot.turnOff();
            }
            if (this.gamepad1.right_stick_x > 0.5) {
                robot.turnForTime(robot_power, 10, false, -1);
            }
            if (this.gamepad1.right_stick_x < -0.5) {
                robot.turnForTime(robot_power, 10, false, 1);
            }


            // Moving
            if (this.gamepad1.right_stick_x < 0.5 && this.gamepad1.right_stick_x > -0.5 && this.gamepad1.right_stick_y < 0.5 && this.gamepad1.right_stick_y > -0.5) {
                robot.turnOff();
            }
            if (this.gamepad1.left_stick_y > 0.5) {
                robot.moveForward(robot_power);
            }
            if (this.gamepad1.left_stick_y < -0.5) {
                robot.moveBackward(robot_power);
            }
            if (this.gamepad1.left_stick_x > 0.5) {
                robot.moveRight(robot_power);
            }
            if (this.gamepad1.left_stick_x < -0.5) {
                robot.moveLeft(robot_power);
            }

            if (this.gamepad1.dpad_left == false && this.gamepad1.dpad_right == false && this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
                robot.turnOff();
            }

            if (this.gamepad1.a == true) {
                robot.pixGrab();
            }

            if (this.gamepad1.b == true) {
                robot.pixRelease();
            }

            if (this.gamepad2.a == true) {
                robot.wrist_grab();
            }

            if (this.gamepad2.x == true) {
                robot.wrist_mid();
            }
            if(this.gamepad2.y == true) {
                robot.wrist_drop();
            }


            if (this.gamepad1.right_trigger > 0.5) {
                robot.armRdy();
            }
            if (this.gamepad1.left_trigger > 0.5) {
                robot.armPark();
            }

            // Use gamepad buttons to move the slide
            // expand (Left Bumper) and contracts (Right Bumper)
            if (this.gamepad1.left_bumper) {
                 robot.expandSlide();
            } else if (this.gamepad1.right_bumper) {
                 robot.contractSlide();
            } else {
                 robot.stopSlide();
            }

            if (this.gamepad2.x == true) {
                // robot.pushPlane();
            }
            if (this.gamepad1.dpad_up == true) {
//                robot.moveForwardToPosition(robot_power, 18, 3);
                robot.turnSlideUp();
                robot.expandSlide();
            }
            if (this.gamepad1.dpad_down == true) {
//                robot.moveBackwardToPosition(robot_power, 18, 3);
                robot.contractSlide();
                robot.turnSlideBack();
            }
            if (this.gamepad1.dpad_left == true) {
//                robot.moveBackwardToPosition(robot_power, 18, 3);
                robot.turnSlideBackSlow();
            }
            if (this.gamepad1.dpad_left == true) {
//                robot.rotateAntiClock(90, robot_power);
//                robot.moveLeftToPosition(robot_power, 18, 3);
            }
            if (this.gamepad1.dpad_right == true) {
//                robot.rotateAntiClock(-90, robot_power);
//                robot.moveRightToPosition(robot_power, 18, 3);
            }

            if (this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
                robot.turnOff();
            }
        };
    };
}
