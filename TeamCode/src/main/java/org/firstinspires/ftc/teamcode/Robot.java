package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot extends Thread {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    double arm_start_position = 0.0;
    double claw_start_position = 1;

    double arm_end_position = .65;
    double claw_end_position = 0.5;

    //private static final int TICKS_PER_ROTATION = 1440; //Tetrix motor specific
    private static final double TICKS_PER_ROTATION = 537.7; //Gobilda 5203 312 RMP motor specific
    private static final double ARM_TICKS_PER_ROTATION = 145.1;
    private static final double WHEEL_DIAMETER = 3.78; //Wheel diameter in inches
    private static final double LA_EXPAND_POWER =  0.80 ; // Run actuator motor up at 80% power
    private static final double LA_CONTRACT_POWER  = -0.80 ; // Run actuator motor down at -80% power
    private String TAG = "FTC";

    private DcMotorEx Motor_FL;
    private DcMotorEx Motor_FR;
    private DcMotorEx Motor_BR;
    private DcMotorEx Motor_BL;
    private DcMotorEx Motor_VSL, Motor_VSR;
    private DcMotorEx Motor_WBL;
    private DcMotorEx Motor_WBR;
    private Servo planePusher, armServo, clawServo;
   // private CRServo clawServo;
    //private Servo armServo;
    private BNO055IMU imu;
    private Orientation     angles;
    private PIDController   pidRotate, pidDrive;
    private Orientation     lastAngles = new Orientation();
    private double          globalAngle, correction;


    private final int tollerance = 15;


    public boolean isTeleOp = true;
    private boolean DEBUG_DEBUG = true;
    private boolean DEBUG_INFO = true;

    Robot(HardwareMap map, Telemetry tel) {
        hardwareMap = map;
        telemetry = tel;
        initDevices();
    }

    /* Calculate Drivetrain PID cofficients */
/*
    private final int motorFLMaxSpeed = 2780;
    double motorFLF = 32767 / (double) motorFLMaxSpeed;
    double motorFLP = 0.1 * motorFLF;
    double mototFLI = 0.1 * motorFLP;

    private final int motorFRMaxSpeed = 2800;
    double motorFRF = 32767 / (double) motorFRMaxSpeed;
    double motorFRP = 0.1 * motorFRF;
    double mototFRI = 0.1 * motorFRP;

    private final int motorBLMaxSpeed = 2760;
    double motorBLF = 32767 / (double) motorBLMaxSpeed;
    double motorBLP = 0.1 * motorBLF;
    double mototBLI = 0.1 * motorBLP;

    private final int motorBRMaxSpeed = 2720;
    double motorBRF = 32767 / (double) motorBRMaxSpeed;
    double motorBRP = 0.1 * motorBRF;
    double mototBRI = 0.1 * motorBRP; */


    private void initDeviceCore() throws Exception {

        telemetry.addData("Please wait", "In function init devices");
        telemetry.update();

        //Wheels
        Motor_FL = hardwareMap.get(DcMotorEx.class, "motor_fl");
        Motor_FR = hardwareMap.get(DcMotorEx.class, "motor_fr");
        Motor_BR = hardwareMap.get(DcMotorEx.class, "motor_br");
        Motor_BL = hardwareMap.get(DcMotorEx.class, "motor_bl");
        Motor_VSL = hardwareMap.get(DcMotorEx.class, "vs_l");
        Motor_VSR = hardwareMap.get(DcMotorEx.class, "vs_r");
        Motor_WBL = hardwareMap.get(DcMotorEx.class, "wb_l");
        Motor_WBR = hardwareMap.get(DcMotorEx.class, "wb_r");

        //planePusher = hardwareMap.get(Servo.class, "planePusher");
        //clawServo   = hardwareMap.get(Servo.class, "clawServo");
        //armServo    = hardwareMap.get(Servo.class, "armServo");
       /*Motor_FR.setVelocityPIDFCoefficients(0.95, 0.095, 0, 9.5);
        Motor_FR.setPositionPIDFCoefficients(5.0);

        Motor_FL.setVelocityPIDFCoefficients(0.95, 0.095, 0, 9.5);
        Motor_FL.setPositionPIDFCoefficients(5.0);

        Motor_BR.setVelocityPIDFCoefficients(0.93, 0.093, 0, 9.3);
        Motor_BR.setPositionPIDFCoefficients(5.0);

        Motor_BL.setVelocityPIDFCoefficients(0.93, 0.093, 0, 9.3);
        Motor_BL.setPositionPIDFCoefficients(5.0);

        Log.i(TAG, "Motor FR Cofficients: P: " + motorFRP + " I: " + mototFRI + " F: " + motorFRF);
        Log.i(TAG, "Motor FL Cofficients: P: " + motorFLP + " I: " + mototFLI + " F: " + motorFLF);
        Log.i(TAG, "Motor BR Cofficients: P: " + motorBRP + " I: " + mototBRI + " F: " + motorBRF);
        Log.i(TAG, "Motor BL Cofficients: P: " + motorBLP + " I: " + mototBLI + " F: " + motorBLF);

        Motor_FR.setVelocityPIDFCoefficients(motorFRP, mototFRI, 0, motorFRF);
        Motor_FR.setPositionPIDFCoefficients(8.2);

        Motor_FL.setVelocityPIDFCoefficients(motorFLP, mototFLI, 0, motorFLF);
        Motor_FL.setPositionPIDFCoefficients(8.0);

        Motor_BR.setVelocityPIDFCoefficients(motorBRP, mototBRI, 0, motorBRF);
        Motor_BR.setPositionPIDFCoefficients(8.2);

        Motor_BL.setVelocityPIDFCoefficients(motorBLP, mototBLI, 0, motorBLF);
        Motor_BL.setPositionPIDFCoefficients(8.0);

        Motor_FL.setTargetPositionTolerance(15);
        Motor_FR.setTargetPositionTolerance(15);
        Motor_BL.setTargetPositionTolerance(15);
        Motor_BR.setTargetPositionTolerance(15);*/


        Motor_FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor_VSL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_VSR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_WBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor_WBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
//        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parametersIMU.calibrationDataFile = "BNO055IMUCalibration.json";
//        parametersIMU.loggingEnabled = false;
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parametersIMU);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.0099, .0001, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Log.i(TAG, "Start Orientation First : "+ angles.firstAngle + "Second: " + angles.secondAngle + "Third: " + angles.thirdAngle );

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
            try {
                sleep(10000);
            } catch (Exception e1) {
            }

        }

    }

    private void pause(int milliSec) {
        try {
            sleep(milliSec);
        } catch (Exception e) {
        }
    }

    // This function takes input distance in inches and will return Motor ticks needed
    // to travel that distance based on wheel diameter
    private int DistanceToTick(double distance) {
        // Log.i(TAG, "Enter FUNC: DistanceToTick");

        double circumference = WHEEL_DIAMETER * 3.14;
        double num_rotation = distance / circumference;
        int encoder_ticks = (int) (num_rotation * TICKS_PER_ROTATION);

        //       Log.i(TAG,"Rotation Needed : " + num_rotation);
        //if (DEBUG_INFO) {
        Log.i(TAG, "Ticks Needed : " + encoder_ticks);
        //  Log.i(TAG, "Exit FUNC: DistanceToTick");
        //}

        return (encoder_ticks);
    }


    boolean drivetrainBusy(int ticks) {
        int avg = (Math.abs(Motor_FL.getCurrentPosition())
                + Math.abs(Motor_FR.getCurrentPosition())
                + Math.abs(Motor_BL.getCurrentPosition())
                + Math.abs(Motor_BR.getCurrentPosition())) / 4;
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
    public void moveForwardToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveForwardToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        long startTime = System.currentTimeMillis();

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

        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
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
    public void moveBackwardToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveBackwardToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        long startTime = System.currentTimeMillis();

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

        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
            if (DEBUG_DEBUG) {
                Log.i(TAG, "Actual Ticks Motor0 : " + Motor_FL.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor1 : " + Motor_FR.getCurrentPosition());
                Log.i(TAG, "Actual Ticks Motor2 : " + Motor_BR.getCurrentPosition());
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
    public void moveLeftToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveLeftToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        long startTime = System.currentTimeMillis();

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

        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
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
    public void moveRightToPosition(double power, double distance) {
        Log.i(TAG, "Enter Function: moveRightToPosition Power : " + power + " and distance : " + distance);
        // Reset all encoders
        long startTime = System.currentTimeMillis();

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

        while (Motor_FL.isBusy() && Motor_BL.isBusy() && Motor_FR.isBusy() && Motor_BR.isBusy()) {
            if ((System.currentTimeMillis() - startTime) > 3000) {
                break;
            }
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

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

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

        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

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


        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

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


        try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

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

       /* try {
            sleep(time);
        } catch (Exception e) {
            e.printStackTrace();
        }

        //Reached the distance, so stop the motors
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);*/
    }

    public void turnOff(){
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BR.setPower(0);
        Motor_BL.setPower(0);
    }


    public void moveF(double power, long distance) {

        Motor_FL.setPower(power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower(power);

    }

    public void moveB(double power, long distance) {
        Motor_FL.setPower((-1) * power); //FL
        Motor_FR.setPower(power); //FR
        Motor_BR.setPower(power); //BR
        Motor_BL.setPower((-1) * power); //BL

    }

    public void moveL(double power, long distance) {
        Motor_FL.setPower(power );
        Motor_FR.setPower(power);
        Motor_BR.setPower((-1) * power);
        Motor_BL.setPower((-1) *  power);
    }

    public void moveR(double power, long distance) {
        Motor_FL.setPower((-1) * power);
        Motor_FR.setPower((-1) * power);
        Motor_BR.setPower(power);
        Motor_BL.setPower(power);

    }


    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

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
          /*  while (getAngle() == 0)
            {
                // set power to rotate.
                Motor_FL.setPower(power);
                Motor_FR.setPower(power);
                Motor_BL.setPower(power);
                Motor_BR.setPower(power);
                Log.i(TAG, "Function: rotate, Angle less then 0 Motor Power set to: " + power);
                try {
                    sleep(100);
                } catch (Exception e) {
                }
            }*/

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
        try {
            sleep(500);
        } catch (Exception e) {
            e.printStackTrace();
        }

        // reset angle tracking on new heading.
        resetAngle();
        Log.i(TAG, "Exit Function: rotate");
    }


    public void pushPlane (){
        planePusher.setDirection(Servo.Direction.REVERSE);
        planePusher.setPosition(0.8);
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        planePusher.setPosition(0.0);
    }

    /* Grab the pixel */
    public void pixRelease() {
//        clawServo.setDirection(FORWARD);
//        clawServo.setPower(.2);
//        try {
//            sleep(10);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        clawServo.setPower(0);
        clawServo.setPosition(claw_end_position);
    }

    /* release the pixel */
    public  void pixGrab(){

//        clawServo.setDirection(REVERSE);
//        clawServo.setPower(.8);
//        try {
//            sleep(10);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        clawServo.setPower(0);
         clawServo.setPosition(claw_start_position);

//        double currPosition = 0;
//        currPosition = clawServo.getPosition();
//        currPosition = currPosition + 0.1;
//        if (currPosition < .2){
//            clawServo.setPosition(currPosition);
//        } else {
//            clawServo.setPosition(0.2);
//        }
//        while (currPosition <= 0.4) {
//            currPosition = currPosition + 0.1;
//            clawServo.setPosition(currPosition);
//            try {
//                sleep(100);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
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
        telemetry.addData("Arm Down", "Position %f ", curr_position); //Displays "Status: Initialized"
        telemetry.update();
        try {
            sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        curr_position = curr_position + 0.1;
        if(curr_position <= 1) {
            armServo.setPosition(curr_position);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else {
            armServo.setPosition(1);
        }
    }

    public void armUp (){
        double curr_position = armServo.getPosition();
        telemetry.addData("Arm Up", "Position %f ", curr_position); //Displays "Status: Initialized"
        telemetry.update();
        try {
            sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        curr_position = curr_position - 0.1;
        if(curr_position >= 0.0) {
            armServo.setPosition(curr_position);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else {
            armServo.setPosition(0.1);
        }
    }

    public void clawSlowOpen (){
        double curr_position = clawServo.getPosition();
        telemetry.addData("ClawOpen", "Position %f ", curr_position); //Displays "Status: Initialized"
        telemetry.update();
        try {
            sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        curr_position = curr_position + 0.1;
        if(curr_position >= claw_end_position) {
            clawServo.setPosition(curr_position);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else {
            clawServo.setPosition(claw_end_position);
        }
    }

    public void clawSlowClose (){
        double curr_position = clawServo.getPosition();
        telemetry.addData("Claw Close", "Position %f ", curr_position); //Displays "Status: Initialized"
        telemetry.update();
        try {
            sleep(10);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        curr_position = curr_position - 0.1;
        if(curr_position <= claw_start_position) {
            clawServo.setPosition(curr_position);
            try {
                sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        } else {
            clawServo.setPosition(claw_start_position);
        }
    }

    public void armOff (){
        armServo.close();
    }

    /* claw to face backdrop and release the pixel */
    public void pixOnBackdrop () {
        armServo.setPosition(0.6);
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        //clawServo.setPosition(0.2);
        //pixRelease();
    }

    public void pixGrip () {
        armServo.setPosition(0.2);
        try {
            sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        clawServo.setPosition(0.1);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        clawServo.setPosition(0);

    }

    /* just a test routine */
    public void clawTest () {
        armRdy();
        pixGrab();
        try {
            sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        armPark();
        try {
            sleep(2000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        moveForwardToPosition(1,24);
        moveRightToPosition(1,23);
        moveForwardToPosition(1, 17);
        pixOnBackdrop();
        try {
            sleep(1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
        moveBackwardToPosition(1,5);
    }

    public void expandSlide () {
        Motor_VSL.setPower(LA_EXPAND_POWER);
        Motor_VSR.setPower(LA_EXPAND_POWER);
        Log.i(TAG, "Slide Expanding");
    }

    public void contractSlide () {
        Motor_VSL.setPower(LA_CONTRACT_POWER);
        Motor_VSR.setPower(LA_CONTRACT_POWER);
        Log.i(TAG, "Slide Contracting");
    }

    public void stopSlide() {
        Motor_VSL.setPower(0.0);
        Motor_VSR.setPower(0.0);
        Log.i(TAG, "Slide Stopped");
    }

    public void turnClaw() {
        Motor_WBL.setPower(1);
        Motor_WBR.setPower(1);

    }
    public void turnClawBack() {
        Motor_WBL.setPower(-1);
        Motor_WBR.setPower(-1);
    }
    public void clawStop() {
        Motor_WBL.setPower(0.0);
        Motor_WBR.setPower(0.0);

        Motor_WBL.setTargetPosition(80);
    }
}
