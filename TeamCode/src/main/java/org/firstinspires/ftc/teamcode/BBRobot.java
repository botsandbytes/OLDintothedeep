 package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.abs;
import android.util.Log;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class BBRobot extends Thread {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    double arm_start_position = 0.0;
    double arm_end_position = .65;
    double claw_end_position = .65;
    double claw_start_position = .8;
    double wrist_end_position = .2;
    double wrist_start_position = 1;

    private final int tollerance = 15;
    private static final double TICKS_PER_ROTATION = 537.7; //Gobilda 5203 312 RMP motor specific
    private static final double WHEEL_DIAMETER = 4.094; //Wheel diameter in inches
    private static final double WHEEL_WIDTH = 1.496; // wheel width in inches
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.

    private static final double ARM_TICKS_PER_ROTATION = 145.1;
    private static final double ARM_WHEEL_DIAMETER = 1; //Wheel diameter in inches
    static final double ARM_COUNTS_PER_INCH = (ARM_TICKS_PER_ROTATION * DRIVE_GEAR_REDUCTION) /
            (ARM_WHEEL_DIAMETER * 3.1415);

    private String TAG = "BnB";

    private DcMotorEx Motor_FL;
    private DcMotorEx Motor_FR;
    private DcMotorEx Motor_BR;
    private DcMotorEx Motor_BL;
    private DcMotorEx Motor_VSL;
    private DcMotorEx Motor_VSR;
    //private DcMotorEx Motor_WBL;
    private DcMotorEx Motor_WBR;
    private Servo armServo, wristServo, clawServo;
    private IMU imu;
    private Orientation angles;
    private PIDController pidRotate, pidDrive;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, correction;
    public boolean isTeleOp = true;
    private boolean DEBUG_DEBUG = true;
    private boolean DEBUG_INFO = true;

    BBRobot(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initDevices();
    }

    private void initDeviceCore() throws Exception {

        telemetry.addData("Please wait", "In function init devices");
        telemetry.update();

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        //Wheels
        Motor_FL = hardwareMap.get(DcMotorEx.class, "motor_fl");
        Motor_FR = hardwareMap.get(DcMotorEx.class, "motor_fr");
        Motor_BR = hardwareMap.get(DcMotorEx.class, "motor_br");
        Motor_BL = hardwareMap.get(DcMotorEx.class, "motor_bl");
        Motor_VSL = hardwareMap.get(DcMotorEx.class, "vs_l");
        Motor_VSR = hardwareMap.get(DcMotorEx.class, "vs_r");
        //Motor_WBL = hardwareMap.get(DcMotorEx.class, "wb_l");
        Motor_WBR = hardwareMap.get(DcMotorEx.class, "wb_r");

        clawServo   = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        //armServo    = hardwareMap.get(Servo.class, "armServo");

        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_VSL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_VSR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Motor_WBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_WBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_VSL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_VSR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        Motor_WBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_WBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.0099, .0001, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);
        //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.i(TAG, "Start Orientation First : "+ angles.firstAngle + "Second: " + angles.secondAngle + "Third: " + angles.thirdAngle );

        telemetry.addData("New Status", "Initialized and Angles are %s - %s - %s", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
        telemetry.update();
        pause(5000);
    }

    private void pause(int milliSec) {
        try {
            sleep(milliSec);
        } catch (Exception e) {
        }
    }

    private void initDevices() {
        ElapsedTime mRuntime;
        mRuntime = new ElapsedTime();
        mRuntime.reset();

        try {
            initDeviceCore();
        } catch (Exception e) {
            telemetry.addData("Exception", "In function init devices" + e);
            telemetry.update();
            pause(10000);
        }
    }

    // This function takes input distance in inches and will return Motor ticks needed
    // to travel that distance based on wheel diameter
    private int DistanceToTick(double distance) {
        double circumference = WHEEL_DIAMETER * 3.14;
        double num_rotation = distance / circumference;
        int encoder_ticks = (int) (num_rotation * TICKS_PER_ROTATION);
        Log.i(TAG, "Ticks Needed : " + encoder_ticks);
        return (encoder_ticks);
    }

    boolean drivetrainBusy(int ticks) {
        int avg = (abs(Motor_FL.getCurrentPosition())
                + abs(Motor_FR.getCurrentPosition())
                + abs(Motor_BL.getCurrentPosition())
                + abs(Motor_BR.getCurrentPosition())) / 4;
        if ((avg >= (ticks - tollerance)) || (avg <= (ticks + tollerance))) {
            return false;
        }
        return true;
    }


    /*****************************************************************************/
    /* Section:      Move to specific distance functions                         */
    /*                                                                           */
    /* Purpose:    Used for moving motor specific inches                         */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     distance      -  in inches                              */
    /*                                                                           */

    /*****************************************************************************/
    // Move forward to specific distance in inches, with power (0 to 1)
    public void moveForwardToPosition(double power, double distance, double timeoutS) {
        Log.i(TAG, "Enter Function: moveForwardToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        runtime.reset();

        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition((-1) * ticks);
        Motor_FR.setTargetPosition(ticks);
        Motor_BR.setTargetPosition(ticks);
        Motor_BL.setTargetPosition((-1) * ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.seconds() < timeoutS) && Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Forward", "Moving");
            telemetry.update();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveForwardToPosition");
        }
    }


    /*****************************************************************************/
    /* Section:      Move to specific distance functions                         */
    /*                                                                           */
    /* Purpose:    Used for moving motor specific inches                         */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     distance      -  in inches                              */
    /*                                                                           */

    /*****************************************************************************/
    // Move backward to specific distance in inches, with power (0 to 1)
    public void moveBackwardToPosition(double power, double distance, double timeoutS) {
        Log.i(TAG, "Enter Function: moveBackwardToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        runtime.reset();

        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition(ticks);
        Motor_FR.setTargetPosition((-1) * ticks);
        Motor_BR.setTargetPosition((-1) * ticks);
        Motor_BL.setTargetPosition(ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.seconds() < timeoutS) && Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Backward", "Moving");
            telemetry.update();
        }


        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveBackwardToPosition");
        }
    }


    /*****************************************************************************/
    /* Section:      Move to specific distance functions                         */
    /*                                                                           */
    /* Purpose:    Used for moving motor specific inches                         */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     distance      -  in inches                              */
    /*                                                                           */

    /*****************************************************************************/
    // Move Left to specific distance in inches, with power (0 to 1)
    public void moveLeftToPosition(double power, double distance, double timeoutS) {
        Log.i(TAG, "Enter Function: moveLeftToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        runtime.reset();

        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition(ticks);
        Motor_FR.setTargetPosition(ticks);
        Motor_BR.setTargetPosition((-1) * ticks);
        Motor_BL.setTargetPosition((-1) * ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.seconds() < timeoutS) && Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Left", "Moving");
            telemetry.update();
        }


        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveLeftToPosition");
        }
    }


    /*****************************************************************************/
    /* Section:      Move to specific distance functions                         */
    /*                                                                           */
    /* Purpose:    Used for moving motor specific inches                         */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     distance      -  in inches                              */
    /*                                                                           */

    /*****************************************************************************/
    // Move Right to specific distance in inches, with power (0 to 1)
    public void moveRightToPosition(double power, double distance, double timeoutS) {
        Log.i(TAG, "Enter Function: moveRightToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        runtime.reset();

        Motor_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Find the motor ticks needed to travel the required distance
        int ticks = DistanceToTick(distance);

        // Set the target position for all motors (in ticks)
        Motor_FL.setTargetPosition((-1) * ticks);
        Motor_FR.setTargetPosition((-1) * ticks);
        Motor_BR.setTargetPosition(ticks);
        Motor_BL.setTargetPosition(ticks);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        //Set Motors to RUN_TO_POSITION
        Motor_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((runtime.seconds() < timeoutS) && Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            }
            //Waiting for Robot to travel the distance
            telemetry.addData("Right", "Moving");
            telemetry.update();
        }


        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        if (DEBUG_INFO) {
            Log.i(TAG, "TICKS needed : " + ticks);
            Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
            Log.i(TAG, "Actual Ticks Motor3 : " + Motor_BL.getCurrentPosition());
            Log.i(TAG, "Exit Function: moveRightToPosition");
        }
    }


    /*****************************************************************************/
    /* Section:      Move For specific time functions                            */
    /*                                                                           */
    /* Purpose:    Used if constant speed is needed                              */
    /*                                                                           */
    /* Returns:   Nothing                                                        */
    /*                                                                           */
    /* Params:    IN     power         - Speed  (-1 to 1)                        */
    /*            IN     time          - Time in MilliSeconds                    */
    /*                                                                           */
    /*****************************************************************************/
    // Move forward for specific time in milliseconds, with power (0 to 1)
    public void moveBackwardForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveBackwardForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Set power of all motors
        Motor_FL.setPower((-1) * power);
        Motor_FR.setPower(power);
        Motor_BR.setPower(power);
        Motor_BL.setPower((-1) * power);

        pause(time);

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveBackwardForTime");
    }

    public void moveForwardForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveForwardForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower(power);

       pause(time);

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveForwardForTime");
    }

    public void moveLeftForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveLeftForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set power of all motors
        Motor_FL.setPower(power);
        Motor_FR.setPower(power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower((-1) * power);

        pause(time);

        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveLeftForTime");
    }

    public void moveRightForTime(double power, int time, boolean speed) {
        Log.i(TAG, "Enter Function: moveRightForTime Power : " + power + " and time : " + time);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set power of all motors
        Motor_FL.setPower((-1) * power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

        pause(time);

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);

        Log.i(TAG, "Exit Function: moveRightForTime");
    }

    public void turnForTime(double power, int time, boolean speed, int orientation) {
        Log.i(TAG, "Enter Function: turnForTime Power : " + power + " and time : " + time + "Speed : " + speed + "orientation : " + orientation);

        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Set power of all motors
        Motor_FL.setPower(orientation * power);
        Motor_FR.setPower(orientation * power);
        Motor_BR.setPower(orientation * power);
        Motor_BL.setPower(orientation * power);
    }

    public void turnOff(){
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }

    public void moveForward(double power) {
        Motor_FL.setPower(power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower(power);
    }

    public void moveBackward(double power) {
        Motor_FL.setPower((-1) * power); //FL
        Motor_FR.setPower(power); //FR
        Motor_BR.setPower(power); //BR
        Motor_BL.setPower((-1) * power); //BL
    }

    public void moveLeft(double power) {
        Motor_FL.setPower(power );
        Motor_FR.setPower(power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower((-1) *  power);
    }

    public void moveRight(double power) {
        Motor_FL.setPower((-1) * power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);
    }

    private void resetAngle()
    {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotateAntiClock(int degrees, double power)
    {
        Log.i(TAG, "Enter Function: rotate, Angle: " + degrees);

        double angle;
        Motor_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            do// Right  turn.
            {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be - on right turn.
                Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
            } while (!pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                angle = getAngle();
                power = pidRotate.performPID(angle); // power will be + on left turn.
                Log.i(TAG, "Function: rotate, Angle More then 0 Motor Power set to: " + power + "Angle : " + angle);
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
            } while (!pidRotate.onTarget());

        // turn the motors off.
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);

        //rotation = getAngle();

        // wait for rotation to stop.
        pause(500);

        // reset angle tracking on new heading.
        resetAngle();
        Log.i(TAG, "Exit Function: rotate");
    }

    /* Grab the pixel */
    public void pixRelease() {
        clawServo.setPosition(claw_start_position);
    }

    /* release the pixel */
    public  void pixGrab(){
         clawServo.setPosition(claw_end_position);
    }

    /* stowed away inside, claw facing backwards */
    public void armPark (){
        armServo.setPosition(.1);
    }

    /* pointing down */
    public void armRdy (){
        armServo.setPosition(1);
    }
    public void armGatePickUp(){
        armServo.setPosition(.83);
    };

    public void armDown (){
        double curr_position = armServo.getPosition();
        telemetry.addData("Arm Down", "Position %f ", curr_position);
        telemetry.update();
        pause(100);
        curr_position = curr_position + 0.1;
        if(curr_position <= 1) {
            armServo.setPosition(curr_position);
            pause(100);
        } else {
            armServo.setPosition(1);
        }
    }

    public void armUp (){
        double curr_position = armServo.getPosition();
        telemetry.addData("Arm Up", "Position %f ", curr_position);
        telemetry.update();
        pause(100);

        curr_position = curr_position - 0.1;
        if(curr_position >= 0.0) {
            armServo.setPosition(curr_position);
            pause(100);
        } else {
            armServo.setPosition(0.1);
        }
    }

    public void wristUp (){
        double curr_position = wristServo.getPosition();
        telemetry.addData("wristOpen", "Position %f ", curr_position);
        telemetry.update();
        pause(100);
        curr_position = curr_position + 0.1;
        if(curr_position >= wrist_end_position) {
            wristServo.setPosition(curr_position);
            pause(100);
        } else {
            wristServo.setPosition(wrist_end_position);
        }
    }

    public void wristDown (){
        double curr_position = wristServo.getPosition();
        telemetry.addData("Wrist Close", "Position %f ", curr_position);
        telemetry.update();
        pause(100);

        curr_position = curr_position - 0.1;
        if(curr_position <= wrist_start_position) {
            wristServo.setPosition(curr_position);
            pause(100);
        } else {
            wristServo.setPosition(wrist_start_position);
        }
    }

    public void armOff (){
        armServo.close();
    }

    /* claw to face backdrop and release the pixel */
    public void pixOnBackdrop () {
        armServo.setPosition(0.6);
        pause(1000);
    }

//    public void pixGrip () {
//        armServo.setPosition(0.2);
//        pause(200);
//        clawServo.setPosition(0.1);
//        pause(100);
//        clawServo.setPosition(0);
//    }

    /* just a test routine */
    public void clawTest () {
        armRdy();
        pixGrab();
        pause(100);
        armPark();
        pause(100);
        moveForwardToPosition(1,24,5);
        moveRightToPosition(1,23, 5);
        moveLeftToPosition(1,23, 5);
        moveForwardToPosition(1, 17, 5);
        pixOnBackdrop();
        pause(100);
        moveBackwardToPosition(1,5, 5);
    }

    public void expandSlide () {
        Log.i(TAG, "Slide Expanding");
        moveSlide(Motor_VSL, Motor_VSR,1,.4,3);
    }

    public void contractSlide () {
        moveSlide(Motor_VSL, Motor_VSR,1,-.2,3);
        Log.i(TAG, "Slide Contracting");
    }

    public void stopSlide() {
//        Motor_VSL.setPower(0.0);
//        Motor_VSR.setPower(0.0);
//        Log.i(TAG, "Slide Stopped");
    }

    public void turnSlide() {
        turnSlide(1,4,2);
    }
    public void turnSlideBack() {
        turnSlide(1,-4,2);
    }
    public void clawStop() {
        //Motor_WBL.setPower(0.0);
        Motor_WBR.setPower(0.0);
    }

    private void moveSlide(DcMotorEx leftMotor,DcMotorEx rightMotor, double speed, double moveDistance, double timeoutS)
    {
        // make motor run using encoder
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setDirection(REVERSE);
        rightMotor.setDirection(FORWARD);

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         // Send telemetry message to indicate the current and new location
        int moveTarget =  (int) (moveDistance * ARM_COUNTS_PER_INCH);
        telemetry.addData("Starting at", "left %7d, right %7d and moving to %7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition(), moveTarget);
        telemetry.update();
        pause(1000);

        //move to new target position
        leftMotor.setTargetPosition(moveTarget);
        rightMotor.setTargetPosition(moveTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);


        // sleep
        while ((runtime.seconds() < timeoutS) && leftMotor.isBusy() && rightMotor.isBusy()) {
            // Display it for the driver.;
            telemetry.addData("Running to", " %7d, left - %7d, right - %7d", moveTarget, leftMotor.getTargetPosition(), rightMotor.getTargetPosition());
            telemetry.addData("Currently at", " left %7d and right %7d ",
                    leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.addData("time now is", " at %7f ", runtime.seconds());
            telemetry.update();
            pause(100);
        }

        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        // Turn off RUN_TO_POSITION
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pause(100);
    }

    private void turnSlide(double speed, double moveDistance, double timeoutS)
    {
        //DcMotorEx leftMotor = Motor_WBL;
        DcMotorEx rightMotor = Motor_WBR;

            // make motor run using encoder
        //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftMotor.setDirection(FORWARD);
        rightMotor.setDirection(FORWARD);

        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate the current and new location
        int moveTarget =  (int) (moveDistance * ARM_COUNTS_PER_INCH);
        telemetry.addData("Starting at", " right %7d and moving to %7d", rightMotor.getCurrentPosition(), moveTarget);
        telemetry.update();
        pause(1000);

        //move to new target position
        //leftMotor.setTargetPosition(moveTarget);
        rightMotor.setTargetPosition(moveTarget);

        //leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        //leftMotor.setPower(abs(speed));
        rightMotor.setPower(abs(speed));

        // sleep
        while ((runtime.seconds() < timeoutS) && rightMotor.isBusy()) {
            // Display it for the driver.
            telemetry.addData("Running to", " %7d,  right - %7d", moveTarget,  rightMotor.getTargetPosition());
            telemetry.addData("Currently at", " right %7d ",
                     rightMotor.getCurrentPosition());
            telemetry.addData("time now is", " at %7f ",
                    runtime.seconds());
            telemetry.update();
            pause(1000);
        }

        // Stop all motion;
        //leftMotor.setPower(0);
        rightMotor.setPower(0);
        // Turn off RUN_TO_POSITION
        //leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
