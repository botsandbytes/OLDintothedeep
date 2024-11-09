package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// http://192.168.43.1:8080/dash

@Config
@TeleOp
//@Disabled
public class nPIDTest extends LinearOpMode {
    DcMotorEx motor;
    static double maxVelocity = 2800.0;
    public static double targetVelocity = maxVelocity * .8;
    public static double P, I, D, F;

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
            motor.setVelocityPIDFCoefficients(P, I, D, F);
            motor.setVelocity(targetVelocity);
            telemetry.addData("target velocity", targetVelocity);
            telemetry.addData("current velocity", motor.getVelocity());
            telemetry.update();
        }
    }
}