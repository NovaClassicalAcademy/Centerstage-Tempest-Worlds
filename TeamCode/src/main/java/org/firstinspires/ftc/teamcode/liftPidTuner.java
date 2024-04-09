package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PIDF_Control;
@Config
@TeleOp
public class liftPidTuner extends OpMode {
    public PIDF_Control controller;
    private DcMotorEx liftL, liftR;
    public static double p = 0.002, i = 0 ,d = 0, f = 0.0001;
    public static int targetPosition = 0;
    @Override
    public void init() {
        controller = new PIDF_Control(p, i, d, f);
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
    }
    @Override
    public void loop() {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        controller.setF(f);
        telemetry.addData("posR", liftR.getCurrentPosition());
        telemetry.addData("posL", liftL.getCurrentPosition());
        double liftPower = controller.calculate(-liftL.getCurrentPosition(), targetPosition);
        liftL.setPower(-liftPower);
        liftR.setPower(-liftPower);
        telemetry.update();
    }
}