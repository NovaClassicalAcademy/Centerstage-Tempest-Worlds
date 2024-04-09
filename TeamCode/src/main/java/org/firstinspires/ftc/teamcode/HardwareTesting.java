package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp

public class HardwareTesting extends OpMode {
    public PIDF_Control controller;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, liftL, liftR;
    private Servo outtake, twist, axonL, axonR, drop1, drop2;
    public double driveMultiplier = 1;
    public static int targetPosition = 0;
    public static double p = 0, i = 0, d = 0, f = 0;
    public ElapsedTime gametime = new ElapsedTime();
    public IMU imu;

    @Override
    public void init() {

        drop1 = hardwareMap.get(Servo.class, "drop1");
        drop2 = hardwareMap.get(Servo.class, "drop2");

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) { // up
            drop1.setPosition(0.5);
            drop2.setPosition(0.5);
        } else if (gamepad1.dpad_down) {
            drop1.setPosition(0.7);
            drop2.setPosition(0.3);
        }

    }
}