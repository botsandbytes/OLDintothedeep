package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
//@Disabled
public class RUEtest extends LinearOpMode {
    public static double pos;
    DcMotorEx motor;

    @Override
    public void runOpMode() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        motor = hardwareMap.get(DcMotorEx.class, "main");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setPower(0.0);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            motor.setVelocity(pos);
            telemetry.addData("target velocity", pos);
            telemetry.addData("current velocity", motor.getVelocity());
            telemetry.update();
        }
    }
}