package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
@Config
@TeleOp

public class halfDrivetrain extends OpMode {
    public PIDF_Control controller;
    public static double p = 0.0023, i = 0 ,d = 0, f = 0.0001;
    public static int targetPosition = 0;
    public Servo outtake, twist, axonL, axonR, drop1, drop2, hammerL, hammerR, drone;
    public DcMotorEx liftL, liftR, intake, frontLeft, frontRight, backLeft, backRight;
    
    public double multiplier = 1;
    private IMU imu;
    private boolean extended = false;

    @Override
    public void init() {
        controller = new PIDF_Control(p, i, d, f);

        frontLeft = hardwareMap.get(DcMotorEx.class, "fl");
        frontRight = hardwareMap.get(DcMotorEx.class, "fr");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl");
        backRight = hardwareMap.get(DcMotorEx.class, "br");

        outtake = hardwareMap.get(Servo.class, "gripper");
        twist = hardwareMap.get(Servo.class, "twist");
        axonL = hardwareMap.get(Servo.class, "angle");
        axonR = hardwareMap.get(Servo.class, "rotation");

        drop1 = hardwareMap.get(Servo.class, "dropR");
        drop2 = hardwareMap.get(Servo.class, "dropL");

        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        hammerL = hardwareMap.get(Servo.class, "hammerL");
        hammerR = hardwareMap.get(Servo.class, "hammerR");
        drone = hardwareMap.get(Servo.class, "drone");

        imu = hardwareMap.get(IMU.class, "imu");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    @Override
    public void loop() {
        double intakePower = gamepad2.left_trigger;
        double outtakePower = -gamepad2.right_trigger;

        if(gamepad1.left_trigger > 0.1) {
            multiplier = 0.6;
        } else {
            multiplier = 1;
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x* 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = ((y + x + rx) / denominator) * multiplier;
        double backLeftPower = ((y - x + rx) / denominator) * multiplier;
        double frontRightPower = ((y - x - rx) / denominator * multiplier);
        double backRightPower = ((y + x - rx) / denominator) * multiplier;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
        if(extended) {
            if (gamepad2.right_stick_y < -0.25 && (gamepad2.right_stick_x >= -0.1 && gamepad2.right_stick_x <= 0.1)){
                twist.setPosition(0.5);
            }
            if (gamepad2.right_stick_x < -0.25 && gamepad2.right_stick_y < -.25){
                twist.setPosition(0.7);
            }
            if (gamepad2.right_stick_x < -0.25 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                twist.setPosition(0.85);

            }
            if (gamepad2.right_stick_x > 0.25 && gamepad2.right_stick_y > 0.25){
                twist.setPosition(0);
            }
            if (gamepad2.right_stick_x > 0.25 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                twist.setPosition(0.15);
            }
            if (gamepad2.right_stick_x >  0.25 && gamepad2.right_stick_y < -0.25){
                twist.setPosition(0.3);
            }
            if (gamepad2.right_stick_x < -0.25 && gamepad2.right_stick_y > 0.25){
                twist.setPosition(1);
            }
            if(!extended) {
                twist.setPosition(0.5);
            }
        }

        if(gamepad2.right_bumper){
            outtake.setPosition(0.4); // drop
        } else if (gamepad2.left_bumper){
            outtake.setPosition(0); // zero
        }
        if(gamepad2.left_stick_y <= -0.1){
            targetPosition +=15;
            if(targetPosition >= 2850){
                targetPosition = 2850;
            }
        } else if(gamepad2.left_stick_y >= 0.1){
            targetPosition -= 15;
            if(targetPosition <= -90){
                targetPosition = -90;
            }
        }
        if(targetPosition >= 750){
            axonR.setPosition(0.40);
            axonL.setPosition(0.43);
            extended = true;
        } else {
            axonR.setPosition(0.13);
            axonL.setPosition(0.5);
            extended = false;
        }
        if(gamepad2.left_trigger > 0.1) { //intake
            intake.setPower(intakePower);
        } else if (gamepad2.right_trigger > 0.1) { //outtake
            intake.setPower(outtakePower);
        } else {
            intake.setPower(0);
        }
        if (gamepad2.dpad_up) { // up
            drop1.setPosition(0.5);
            drop2.setPosition(0.5);
        } else if (gamepad2.dpad_down) {
            drop1.setPosition(0.7);
            drop2.setPosition(0.3);
        }
        if(gamepad1.dpad_up){
            drone.setPosition(0.4); // drop
        } else if (gamepad1.dpad_down){
            drone.setPosition(1); // zero
        }

        double liftPower = controller.calculate(-liftL.getCurrentPosition(), targetPosition);

        liftL.setPower(-liftPower);
        liftR.setPower(-liftPower);

        telemetry.update();
    }
}