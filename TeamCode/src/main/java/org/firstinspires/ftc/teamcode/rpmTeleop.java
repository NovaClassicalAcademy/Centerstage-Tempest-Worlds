/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @author CyanCheeah
 * This is the TeleOp but for the blue side
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class rpmTeleop extends LinearOpMode
{
    private DcMotor frontl   = null;  //  Used to control the left front drive wheel
    private DcMotor frontr  = null;  //  Used to control the right front drive wheel
    private DcMotor bottoml    = null;  //  Used to control the left back drive wheel
    private DcMotor bottomr   = null;
    private DcMotor rightLift = null;
    private DcMotor leftLift = null;
    private static Servo DroneLauncher = null;

    private static CRServo IntakeUno = null;
    private static CRServo IntakeDos = null;
    private static CRServo IntakeRoller = null;
    private static Servo IntakePos = null;
    private static Servo OuttakeClaw = null;
    private static Servo OuttakeFlip = null;

    private static Servo OuttakeSpin = null;
    private static double MOTOR_ADJUST = 0.75;
    @Override public void runOpMode()
    {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        frontl = hardwareMap.get(DcMotor.class, "fl");
        frontr = hardwareMap.get(DcMotor.class, "fr");
        bottoml = hardwareMap.get(DcMotor.class, "bl");
        bottomr = hardwareMap.get(DcMotor.class, "br");
        leftLift = hardwareMap.get(DcMotor.class,"liftL");
        rightLift = hardwareMap.get(DcMotor.class,"liftR");

        DroneLauncher = hardwareMap.get(Servo.class, "drone");
        IntakeUno = hardwareMap.get(CRServo.class, "IntakeLeft");
        IntakeDos = hardwareMap.get(CRServo.class, "IntakeRight");
        IntakeRoller = hardwareMap.get(CRServo.class, "IntakeRoller");
        IntakePos = hardwareMap.get(Servo.class, "IntakePos");
        OuttakeClaw = hardwareMap.get(Servo.class, "OuttakeClaw");
        OuttakeFlip = hardwareMap.get(Servo.class, "OuttakeFlip");
        OuttakeSpin = hardwareMap.get(Servo.class, "OuttakeSpin");


        //Setting Directions of motors.
        frontl.setDirection(DcMotor.Direction.FORWARD);
        bottoml.setDirection(DcMotor.Direction.FORWARD);
        frontr.setDirection(DcMotor.Direction.REVERSE);
        bottomr.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        //Brake immedietly after joystick hits 0 instead of coasting down
        frontl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottoml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        Constants constants = new Constants();
        double triggerPowerAdjust = 1;
        double speedAdjust = 1.4;
        double intakeUp = constants.getIntakeUp();
        double intakeDown = constants.getIntakeDown();
        double outClose = constants.getOutClose();
        double outOpen = constants.getOutOpen();
        double flipOut = constants.getFlipOut();
        double flipIn = constants.getFlipIn();
        double Top = constants.getSpinTop();
        double topLeft = constants.getSpinTopLeft();
        double left = constants.getSpinLeft();
        double bottomRight = constants.getSpinBottomRight();
        double right = constants.getSpinRight();
        double topRight = constants.getSpinTopRight();
        double bottomLeft = constants.getSpinBottomLeft();
        double intakeSemiUp = constants.getIntakeSemiUp();
        double intakeSemiDown = constants.getIntakeSemiDown();
        int stuff = 0;
        while (opModeIsActive())
        {

            //elevator height code
            if (gamepad2.left_stick_y > 0.3) {
                rightLift.setPower((.25));
                leftLift.setPower((-.25));
            }
            else if (gamepad2.left_stick_y < -0.3) {
                rightLift.setPower((-.25));
                leftLift.setPower((.25));
            }
            else if (gamepad2.dpad_down) {
                rightLift.setPower((.65));
                leftLift.setPower((-.65));
            }
            else if (gamepad2.dpad_up) {
                rightLift.setPower((-.65));
                leftLift.setPower((.65));
            }
            else {
                rightLift.setPower((0));
                leftLift.setPower((0));
            }


            //pixels position code

            if (gamepad2.right_stick_y < -0.5 && (gamepad2.right_stick_x >= -0.1 && gamepad2.right_stick_x <= 0.1)){
                telemetry.addData("Top", stuff);
                OuttakeSpin.setPosition(Top);
            }
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y < -.55){
                telemetry.addData("TopLeft", stuff);
                OuttakeSpin.setPosition(topLeft);

            }
            if (gamepad2.right_stick_x < -0.5 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                telemetry.addData("Left", stuff);
                OuttakeSpin.setPosition(left);

            }
            if (gamepad2.right_stick_x > 0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomRight", stuff);
                OuttakeSpin.setPosition(bottomRight);

            }
            if (gamepad2.right_stick_x > 0.55 && (gamepad2.right_stick_y >= -0.1 && gamepad2.right_stick_y <= 0.1)){
                telemetry.addData("Right", stuff);
                OuttakeSpin.setPosition(right);

            }
            if (gamepad2.right_stick_x >  0.55 && gamepad2.right_stick_y < -.55){
                telemetry.addData("TopRight", stuff);
                OuttakeSpin.setPosition(topRight);

            }
            if (gamepad2.right_stick_x < -0.55 && gamepad2.right_stick_y > 0.55){
                telemetry.addData("BottomLeft", stuff);
                OuttakeSpin.setPosition(bottomLeft);

            }
            if (true) {
                telemetry.addData("rightX", gamepad2.right_stick_x);
                telemetry.addData("rightY", gamepad2.right_stick_y);
            }


            //intake code
            if (gamepad2.x) {
                IntakeUno.setPower((.8));
                IntakeDos.setPower((-.8));
                IntakeRoller.setPower((-1));
            }
            else if (gamepad2.y) {
                IntakeUno.setPower((-.8));
                IntakeDos.setPower((.8));
                IntakeRoller.setPower((1));
            }
            else {
                IntakeUno.setPower((0));
                IntakeDos.setPower((0));
                IntakeRoller.setPower((0));
            }
            //outtake position
            if (gamepad2.dpad_left) {
                OuttakeSpin.setPosition(left);
            }
            if (gamepad2.dpad_right) {
                OuttakeSpin.setPosition(Top);
            }
            //sequence for the outtake flipping
            if (gamepad2.left_bumper) {
                OuttakeFlip.setPosition(flipIn);
                OuttakeSpin.setPosition(Top);
                OuttakeClaw.setPosition(outOpen);
            }
            if (gamepad2.right_bumper) {
                OuttakeFlip.setPosition(flipOut);
            }

            if (gamepad2.left_trigger > 0.5) {
                OuttakeClaw.setPosition(outClose);
            }
            if (gamepad2.right_trigger > 0.5) {
                OuttakeClaw.setPosition(outOpen);
            }
            //          ************************************************ GAMEPAD 1 CONTROLS ************************************************
            /**
             * Gamepad 1 Controls:
             * Dpad Up: DrownUp
             * Dpad Down: DroneDown
             * Dpad Left: NOTHING
             * Dpad Right: NOTHING
             * Left Joystick: NOTHING
             * - Up: Forward
             * - Down: Backwards
             * - Left: Strafe Left
             * - Right: Strafe Right
             * Right Joystick:
             * - Left: Turn in position left
             * - Right: Turn in position right
             * X: intakeUp
             * Y: intakeSemiUp
             * A: IntakePosDown
             * B: intakeSemiDown
             * Left Bumper & Right Bumper: Drone Shoot
             * Right Trigger: Trigger Power Adjust: Slows the robot down by given amount
             * Left Trigger: Makes Robot Faster
             */

            //slows down
            if (gamepad1.right_trigger > 0) {
                triggerPowerAdjust = .4;
            } else {
                triggerPowerAdjust = 1;
            }
            //speeds up
            if (gamepad1.left_trigger > 0) {
                speedAdjust = 1.5;
            } else {
                speedAdjust = 1;
            }

            if(gamepad1.a){
                IntakePos.setPosition(intakeDown);
            }
            if(gamepad1.b){
                IntakePos.setPosition(intakeSemiDown);
            }
            if(gamepad1.y){
                IntakePos.setPosition(intakeSemiUp);
            }
            if(gamepad1.x){
                IntakePos.setPosition(intakeUp);
            }

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1 = (v1 * triggerPowerAdjust * -1) * speedAdjust;
            v2 = (v2 * triggerPowerAdjust * -1) * speedAdjust;
            v3 = (v3 * triggerPowerAdjust * -1) * speedAdjust;
            v4 = (v4 * triggerPowerAdjust * -1) * speedAdjust;
            frontl.setPower(v1 * 1);
            frontr.setPower(v2 * 1);
            bottoml.setPower(v3 * 1);
            bottomr.setPower(v4 * 1);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontl.setPower(leftFrontPower);
        frontr.setPower(rightFrontPower);
        bottoml.setPower(leftBackPower);
        bottomr.setPower(rightBackPower);
    }

}