package org.firstinspires.ftc.teamcode;
//Fix if detecting 2 or 0 minerals
//Give Power to Servo Motor holder
//Buttons to move latch and slide
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopTest extends LinearOpMode {

    @Override
    public void runOpMode()  {
        Robot robot = new Robot(hardwareMap, telemetry);
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
                robot.turnForTime(0.8, 10, false, -1);
            }

            if (this.gamepad1.right_stick_x < -0.5) {
                robot.turnForTime(0.8, 10, false, 1);
            }


            // Moving
            if (this.gamepad1.right_stick_x < 0.5 && this.gamepad1.right_stick_x > -0.5 && this.gamepad1.right_stick_y < 0.5 && this.gamepad1.right_stick_y > -0.5) {
                robot.turnOff();
            }
            if (this.gamepad1.left_stick_y > 0.5) {
                robot.moveF(1, 10);
            }

            if (this.gamepad1.left_stick_y < -0.5) {
                robot.moveB(1, 10);
            }

            if (this.gamepad1.left_stick_x > 0.5) {
                robot.moveR(1, 10);
            }

            if (this.gamepad1.left_stick_x < -0.5) {
                robot.moveL(1, 10);
            }

            /*if (this.gamepad1.left_trigger > 0.5) {
                robot.startIntake(10);
                robot.startShoot();
                robot.teleOpMotorBehavior();
            }

            if (this.gamepad1.b == true) {
                robot.startShoot();
            }
            if (this.gamepad1.b == false){
                robot.endShoot();
            }
            if (this.gamepad2.a == true){
                robot.openGrip();
            }
            if (this.gamepad2.b == true){
                robot.closeGrip();
            }
            if (this.gamepad2.dpad_up){robot.minRaise();}
            if (this.gamepad2.dpad_down){robot.minLower();}
            if(this.gamepad2.dpad_down == false && this.gamepad2.dpad_up == false){robot.stopWobble();}
            if(this.gamepad1.right_trigger > 0.5){
                robot.weakShot();
                robot.stopIntake();
            }*/
            if (this.gamepad1.dpad_left == false && this.gamepad1.dpad_right == false && this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
                robot.turnOff();
            }

//            if (this.gamepad1.dpad_left == true) {
//                robot.moveR(0.5, 10);
//            }
//
//            if (this.gamepad1.dpad_right == true) {
//                robot.moveL(0.5, 10);
//            }

//            if (this.gamepad1.dpad_up == true) {
//                robot.moveF(0.5, 10);
//            }
//
//            if (this.gamepad1.dpad_down == true) {
//                robot.moveB(0.5, 10);
//            }

            if (this.gamepad1.a == true) {
                robot.pixOnBackdrop();
            }

            if (this.gamepad1.b == true) {
                robot.armGatePickUp();
            }

            if (this.gamepad1.right_trigger > 0.5) {
                robot.armRdy();
            }
            if (this.gamepad1.left_trigger > 0.5) {
                robot.armPark();
            }

//            if (this.gamepad1.dpad_up == true) {
//                robot.armDown();
//            } //else
//            if (this.gamepad1.dpad_down == true) {
//                robot.armUp();
//            }
//            else {
//                robot.armOff();
//            }

//            if (this.gamepad1.right_bumper == true) {
//               // robot.clawTest();
//            }

            // Use gamepad buttons to move the linear actuator
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
                robot.turnSlide();
            }
            if (this.gamepad1.dpad_down == true) {
                robot.turnSlideBack();
            }
            if (this.gamepad1.dpad_up == false && this.gamepad1.dpad_down == false) {
                robot.clawStop();
            }
            /* Claw/Arm controls */

//            if (this.gamepad2.dpad_up == true) {
//                robot.clawSlowOpen();
//            } //else
//            if (this.gamepad2.dpad_down == true) {
//                robot.clawSlowClose();
//            }
//
//            if (this.gamepad2.b == true) {
//                robot.pixRelease();
//            }
//            if (this.gamepad2.a == true) {
//                robot.pixGrab();
//            }
//
//            if (this.gamepad2.left_stick_button == true) {
//                robot.pixGrip();
//            }
        };
    };
}
