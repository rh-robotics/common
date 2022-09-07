package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HWC {
    // Robot variables
    public DcMotorEx frontL, frontR, backL, backR, duckWheel, arm, intakeLift;
    public DcMotor extender;
    public CRServo intakeL, intakeR;
    public DistanceSensor dSensorR;
    Telemetry telemetry;
    ElapsedTime waitTime = new ElapsedTime();

    // Code variables
    boolean readingDuck = false;
    public static final double ONE_CM_IN_PPR = 7.9;
    public static final double ARM_VELOCITY = 20000;
    public static final double ONE_DEGREE_IN_PPR = 4.27;
    public static final double EXTENDER_POWER = 0.5;

    public HWC(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Declare all our motors and servos
        duckWheel = hardwareMap.get(DcMotorEx.class, "duckWheel");
        frontL = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        backL = hardwareMap.get(DcMotorEx.class, "leftRear");
        backR = hardwareMap.get(DcMotorEx.class, "rightRear");
        extender = hardwareMap.get(DcMotor.class, "extender");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
//        dSensorR = hardwareMap.get(DistanceSensor.class, "dSensorR");
        intakeLift = hardwareMap.get(DcMotorEx.class, "intakeLift");

        // Set the direction of all our motors and servos
        frontL.setDirection(DcMotorEx.Direction.FORWARD);
        backL.setDirection(DcMotorEx.Direction.FORWARD);
        frontR.setDirection(DcMotorEx.Direction.REVERSE);
        backR.setDirection(DcMotorEx.Direction.REVERSE);
        extender.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        intakeL.setDirection(CRServo.Direction.REVERSE);
        intakeR.setDirection(CRServo.Direction.FORWARD);
        intakeLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // Run motors using encoder, so that we can move accurately. If motor doesn't have, run without encoder
        frontL.setMode(RUN_USING_ENCODER);
        frontR.setMode(RUN_USING_ENCODER);
        backL.setMode(RUN_USING_ENCODER);
        backR.setMode(RUN_USING_ENCODER);
        arm.setMode(RUN_USING_ENCODER);
        extender.setMode(RUN_WITHOUT_ENCODER);
        intakeLift.setMode(RUN_USING_ENCODER);
    }

    // Declare all methods used for moving all parts of our robot.
    public void driveAndArm(double distanceInCm, double wheelRPower, double wheelLPower, double aCounts) {
        int wheelCounts = 0;
        double wCounts = distanceInCm * ONE_CM_IN_PPR;
        int armCounts = 0;
        double armPower = -1;

        frontL.setMode(STOP_AND_RESET_ENCODER);
        frontR.setMode(STOP_AND_RESET_ENCODER);
        backL.setMode(STOP_AND_RESET_ENCODER);
        backR.setMode(STOP_AND_RESET_ENCODER);
        arm.setMode(STOP_AND_RESET_ENCODER);

        frontL.setMode(RUN_WITHOUT_ENCODER);
        frontR.setMode(RUN_WITHOUT_ENCODER);
        backL.setMode(RUN_WITHOUT_ENCODER);
        backR.setMode(RUN_WITHOUT_ENCODER);
        arm.setMode(RUN_WITHOUT_ENCODER);


        waitTime.reset();

        while((Math.abs(wheelCounts) < wCounts) || (Math.abs(armCounts) < aCounts)){
            wheelCounts = frontL.getCurrentPosition();
            armCounts = arm.getCurrentPosition();

            if(Math.abs(wheelCounts) < wCounts){
                frontL.setPower(wheelLPower);
                backL.setPower(wheelLPower);
                frontR.setPower(wheelRPower);
                backR.setPower(wheelRPower);

            }
            else {
                frontL.setPower(0);
                backL.setPower(0);
                frontR.setPower(0);
                backR.setPower(0);
            }

            if(Math.abs(armCounts) < aCounts){
                arm.setPower(armPower);

            }
            else {
                arm.setPower(0);
            }
        }

        frontL.setPower(0);
        backL.setPower(0);
        frontR.setPower(0);
        backR.setPower(0);
        arm.setPower(0);
    }

    public void turn(double directionInDegrees, double wheelVelocity){
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//        4.27(PPR) = 1 Degree
        final double oneDegreeInPPR = 4.27;
        double pprTurn = directionInDegrees * oneDegreeInPPR;

        if(directionInDegrees != 0) {
            if (directionInDegrees < 0) {
                frontL.setTargetPosition(-(int) pprTurn + frontL.getCurrentPosition());
                frontR.setTargetPosition((int) pprTurn + frontR.getCurrentPosition());
                backL.setTargetPosition(-(int) pprTurn + backL.getCurrentPosition());
                backR.setTargetPosition((int) pprTurn + backR.getCurrentPosition());

            } else if (directionInDegrees > 0) {
                frontL.setTargetPosition((int) pprTurn + frontL.getCurrentPosition());
                frontR.setTargetPosition(-(int) pprTurn + frontR.getCurrentPosition());
                backL.setTargetPosition((int) pprTurn + backL.getCurrentPosition());
                backR.setTargetPosition(-(int) pprTurn + backR.getCurrentPosition());
            }

            frontL.setMode(RUN_TO_POSITION);
            frontR.setMode(RUN_TO_POSITION);
            backL.setMode(RUN_TO_POSITION);
            backR.setMode(RUN_TO_POSITION);

            frontL.setVelocity(wheelVelocity);
            frontR.setVelocity(wheelVelocity);
            backR.setVelocity(wheelVelocity);
            backL.setVelocity(wheelVelocity);
        }
    }

    public void duckWheel(double ms){
        waitTime.reset();

        while(waitTime.milliseconds() < ms){
            duckWheel.setPower(0.5);
        }

        duckWheel.setPower(0);
    }

    public void extenderOut(){
        waitTime.reset();

        while(waitTime.milliseconds() < 1200){
            extender.setPower(EXTENDER_POWER);
        }
        extender.setPower(0);
    }

    public void extenderIn(){
        waitTime.reset();

        while(waitTime.milliseconds() < 1200){
            extender.setPower(EXTENDER_POWER);
        }
    }

    public void dropBlock(){
        waitTime.reset();
        while(waitTime.milliseconds() < 1000){
            intakeR.setPower(-1);
            intakeL.setPower(-1);
        }

        intakeR.setPower(0);
        intakeL.setPower(0);
    }

    public void armMiddleLayer(){
        arm.setTargetPosition(-12000);
        arm.setMode(RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY);
        while (arm.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors to Finish Turning");
            telemetry.addData("Motors", "Arm Position: %d, %d", arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }
    }

    public void armBottomLayer(){
        arm.setTargetPosition(-7486);
        arm.setMode(RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY);
        while (arm.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors to Finish Turning");
            telemetry.addData("Motors", "Arm Position: %d, %d", arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }
    }
}
