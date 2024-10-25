package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopTest extends LinearOpMode {
    //private final double robot_power = 1.0;
    BBRobot robot;
    enum DriveState {
        MOVING,
        ARM_CONTROL
    };
    private DriveState currentState = DriveState.MOVING;

    private void initRobot(){
        robot.setWristPosition(0.17);
        robot.turnSlideBack();
    }
    @Override
    public void runOpMode() {

        robot = new BBRobot(hardwareMap, telemetry);
        robot.isTeleOp = true;

        telemetry.addData("Status", "Initialized"); //Displays "Status: Initialized"
        telemetry.update();
        initRobot();
        //Driver must press INIT and then ▶️
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("TeleOP", "New Code");
            telemetry.update();
            switch(currentState) {
                case MOVING:
                    handleMovement();
                    if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 &&
                        gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0)
                    {
                        currentState = DriveState.ARM_CONTROL;
                    }
                    break;
                case ARM_CONTROL:
                    handleControl();
                    if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ||
                            gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0)
                    {
                        currentState = DriveState.MOVING;
                    }
                    break;
            }
        }
    }

    private void handleMovement() {
        /********** GamePad 1 ****************/
        //Turning
        if (this.gamepad1.left_stick_x < 0.5 && this.gamepad1.left_stick_x > -0.5) {
            robot.turnOff();
        }
        if (this.gamepad1.right_stick_x > 0.5) {
            robot.turnForTime(robot.robot_power, 10, false, -1);
        }
        if (this.gamepad1.right_stick_x < -0.5) {
            robot.turnForTime(robot.robot_power, 10, false, 1);
        }

        // Moving
        if (this.gamepad1.right_stick_x < 0.5 && this.gamepad1.right_stick_x > -0.5 && this.gamepad1.right_stick_y < 0.5 && this.gamepad1.right_stick_y > -0.5) {
            robot.turnOff();
        }
        if (this.gamepad1.left_stick_y > 0.5) {
            robot.moveForward(robot.robot_power);
        }
        if (this.gamepad1.left_stick_y < -0.5) {
            robot.moveBackward(robot.robot_power);
        }
        if (this.gamepad1.left_stick_x > 0.5) {
            robot.moveRight(robot.robot_power);
        }
        if (this.gamepad1.left_stick_x < -0.5) {
            robot.moveLeft(robot.robot_power);
        }
    }

    private void handleControl() {
        if (this.gamepad1.dpad_left == false && this.gamepad1.dpad_right == false && this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
            robot.turnOff();
        }

        if (this.gamepad1.a == true) {
            robot.elementGrab();
        }

        if (this.gamepad1.b == true) {
            robot.dropElement();
        }

        if (this.gamepad1.x == true) {
            robot.setWristPosition(0.075);
            //robot.wrist_grab();
        }
        if(this.gamepad1.y == true) {
            robot.wrist_grab();
        }

        if (this.gamepad1.right_trigger > 0.5) {
            robot.hangElementOnHighBar(robot.robot_power);
        }
        if (this.gamepad1.left_trigger > 0.5) {
//                robot.armPark();
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

        if (this.gamepad1.dpad_up == true) {
            robot.turnSlideForDrop();
        }
        if (this.gamepad1.dpad_down == true) {
            robot.turnSlideBack();
        }
        if (this.gamepad1.dpad_left == true) {
            robot.contractSlideSlowRealtively(5);
        }
        if (this.gamepad1.dpad_right == true) {
            robot.expandSlideSlowRealtively(5);
        }

        if (this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
            robot.turnOff();
        }

        /********** GamePad 2 ****************/
        // Code functions for gamepad 2

        if (this.gamepad2.a == true) {
            robot.wrist_grab();
        }
        if (this.gamepad2.x == true) {
            robot.wrist_mid();
        }
        if(this.gamepad2.y == true) {
            robot.wrist_end();
        }

        if (this.gamepad2.dpad_left == true) {
            robot.turnSlideSlowRealtively(5);
        }
        if (this.gamepad2.dpad_right == true) {
            robot.turnSlideSlowRealtively(-5);
        }

        if(this.gamepad2.right_bumper == true) {
            robot.contractSlideNeg();
        }

        if (this.gamepad2.right_trigger > 0.5) {
            robot.turnSlide(robot.robot_power,65, 1500, false );
        }
    };
}