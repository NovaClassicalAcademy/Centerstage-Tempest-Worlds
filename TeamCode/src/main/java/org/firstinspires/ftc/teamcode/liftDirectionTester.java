package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class liftDirectionTester extends OpMode {
    public DcMotorEx liftL, liftR;
    @Override
    public void init() {
        liftL = hardwareMap.get(DcMotorEx.class, "liftL");
        liftR = hardwareMap.get(DcMotorEx.class, "liftR");
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y <  -0.1) {
            liftL.setPower(0.1);
            liftR.setPower(0.1);
        } else if(gamepad1.left_stick_y > 0.1) {
            liftL.setPower(-0.1);
            liftR.setPower(-0.1);
        } else {
            liftL.setPower(0);
            liftR.setPower(0);
        }
        telemetry.addData("LiftLPos", liftL.getCurrentPosition());
        telemetry.addData("LiftRPos", liftR.getCurrentPosition());
        telemetry.addData("joystickLPos", gamepad1.left_stick_y);
        telemetry.update();
    }
}
