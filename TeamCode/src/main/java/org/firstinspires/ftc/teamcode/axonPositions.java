package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tele-Op")
public class axonPositions extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo axon = hardwareMap.servo.get("outtake");


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //Axon
            /*if(gamepad1.left_trigger > 0.1){
                axon.setPosition(0); zero
            } else if (gamepad1.right_trigger > 0.1){
                axon.setPosition(0.7); drop
            }
             */
            //Gobilda intake pivot
            /*if(gamepad1.left_trigger > 0.1){
                axon.setPosition(0.65); //drop
            } else if (gamepad1.right_trigger > 0.1){
                axon.setPosition(1); //zero
            }

             */
             /*
            //Gobilda outtake open/close
            if(gamepad1.left_trigger > 0.1){
                axon.setPosition(0.4); // drop
            } else if (gamepad1.right_trigger > 0.1){
                axon.setPosition(1); // zero
            }

              */

            /*if (gamepad1.dpad_up) {
                axon.setPosition(0.5); // zero
            } else if (gamepad1.dpad_left) {
                axon.setPosition(0.3); // top right
            } else if (gamepad1.b) {
                axon.setPosition(0); // bottom right
            } else if (gamepad1.dpad_right) {
                axon.setPosition(0.7); //top left
            } else if (gamepad1.a) {
                axon.setPosition(1); //bottom left
            } else if (gamepad1.y) {
                axon.setPosition(0.85); //middle
            }
             */
            //launcher
            if(gamepad1.left_trigger > 0.1){
                axon.setPosition(0.4); // drop
            } else if (gamepad1.right_trigger > 0.1){
                axon.setPosition(1); // zero
            }

        }
    }
}