package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.PARK_CENTER;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_BD_ONE_A;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_BD_ONE_B;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_BD_THREE_A;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_BD_THREE_B;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_BD_TWO_A;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_BD_TWO_B;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_SPIKE_ONE;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_SPIKE_THREE;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.W_SPIKE_TWO_ALT;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.TwoCamAprilTagDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous
public class blueW0 extends OpMode {
    public VisionPortal visionPortal1, visionPortal2;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    public AprilTagProcessor aprilTagBack, aprilTagFront;
    public static double Pi = Math.PI;
    public Servo drone;
    public DcMotorEx intake;
    Action toSpikeL, toBDFromSpikeL, toSpikeM, toBDFromSpikeM, toSpikeR, toBDFromSpikeR, toParkFromWL, toParkFromWM, toParkFromWR;

    public class Grip {
        private Servo gripper;

        public Grip(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "gripper");
        }
        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.45);
                return false;
            }
        }
        public Action open() {
            return new Open();
        }
        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0);
                return false;
            }
        }
        public Action close() {
            return new Close();
        }
    }
    public class pitchingIntake {
        private Servo drop1, drop2;
        public pitchingIntake(HardwareMap hardwareMap) {
            drop1 = hardwareMap.get(Servo.class, "dropR");
            drop2 = hardwareMap.get(Servo.class, "dropL");
        }
        public class Up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drop1.setPosition(0.5);
                drop2.setPosition(0.5);
                return false;
            }
        }
        public Action up() {
            return new Up();
        }
        public class Down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                drop1.setPosition(0.7);
                drop2.setPosition(0.3);
                return false;
            }
        }
        public Action down(){
            return new Down();
        }
    }
    public class Hammers{
        private Servo hammerL, hammerR;
        public Hammers(HardwareMap hardwareMap) {
            hammerL = hardwareMap.get(Servo.class, "hammerR");
            hammerR = hardwareMap.get(Servo.class, "hammerL");
        }
        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hammerL.setPosition(0.7);
                hammerR.setPosition(0.3);
                return false;
            }
        }
        public Action intake() {
            return new Intake();
        }
        public class Zero implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hammerL.setPosition(0);
                hammerR.setPosition(1);
                return false;
            }
        }
        public Action zero() {
            return new Zero();
        }
    }
    public class Lift {
        private DcMotorEx liftL, liftR;

        public Servo twist, axonL, axonR;

        public Lift(HardwareMap hardwareMap) {
            liftL = hardwareMap.get(DcMotorEx.class, "liftL");
            liftR = hardwareMap.get(DcMotorEx.class, "liftR");
            twist = hardwareMap.get(Servo.class, "twist");
            axonL = hardwareMap.get(Servo.class, "angle");
            axonR = hardwareMap.get(Servo.class, "rotation");

            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.REVERSE);

            liftR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(0.1);
                    liftR.setPower(0.1);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1100) {
                    return true;
                } else {
                    liftR.setPower(0.1);
                    liftL.setPower(0.1);
                    if(pos > 1000) {
                        axonR.setPosition(0.13);
                        axonL.setPosition(0.5);
                        twist.setPosition(0.50);
                    }
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    twist.setPosition(0.5);
                    axonR.setPosition(0.40);
                    axonL.setPosition(0.43);
                    liftL.setPower(-0.1);
                    liftR.setPower(-0.1);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 40) {
                    return true;
                } else {
                    liftL.setPower(0);
                    liftR.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }
    }

    @Override
    public void init() {
        Grip gripper = new Grip(hardwareMap);
        Hammers hammers = new Hammers(hardwareMap);
        pitchingIntake pitchingIntake = new pitchingIntake(hardwareMap);

        Scalar lower = new Scalar(90, 50, 70); // the lower hsv threshold for Blue
        Scalar upper = new Scalar(128, 255, 255); // the upper hsv threshold for Blue

        double minArea = 200;

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );

        aprilTagFront = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setLensIntrinsics(504.041, 504.041, 307.462, 234.687)
                .build();
        aprilTagBack = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setLensIntrinsics(504.041, 504.041, 307.462, 234.687)
                .build();
        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colourMassDetectionProcessor)
                .build();
        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTagBack)
                .build();

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        drone = hardwareMap.get(Servo.class, "drone");

        TwoCamAprilTagDrive rejat = new TwoCamAprilTagDrive(hardwareMap, new Pose2d(18.50, -63, Math.toRadians(90)), aprilTagBack, aprilTagFront);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        rejat.pose = new Pose2d(-39.5, 63, Math.toRadians(-90));

        //BD RED LEFT 2+0 Auto (Park in Corner)
        toSpikeL = rejat.actionBuilder(rejat.pose)
                .splineToLinearHeading(W_SPIKE_ONE, Pi/2)
                .build();
        toBDFromSpikeL = rejat.actionBuilder(W_SPIKE_ONE)
                .splineToLinearHeading(W_BD_ONE_A, Pi/2)
                .splineToLinearHeading(W_BD_ONE_B, Pi/2)
                .build();
        toParkFromWL = rejat.actionBuilder(W_BD_ONE_B)
                .splineToLinearHeading(PARK_CENTER, Pi/2)
                .build();
        //BD Red Middle 2+0 Auto (Park in Corner)
        toSpikeM = rejat.actionBuilder(rejat.pose)
                .splineToLinearHeading(W_SPIKE_TWO_ALT, Pi/2)
                .build();
        toBDFromSpikeM = rejat.actionBuilder(W_SPIKE_TWO_ALT)
                .splineToLinearHeading(W_BD_TWO_A, Pi/2)
                .splineToLinearHeading(W_BD_TWO_B, Pi/2)
                .build();
        toParkFromWM = rejat.actionBuilder(W_BD_TWO_B)
                .splineToLinearHeading(PARK_CENTER, Pi/2)
                .build();
        //BD Red Right 2+0 Auto (Park in Corner)
        toSpikeR = rejat.actionBuilder(rejat.pose)
                .splineToLinearHeading(W_SPIKE_THREE, Pi/2)
                .build();
        toBDFromSpikeR = rejat.actionBuilder(W_SPIKE_THREE)
                .splineToLinearHeading(W_BD_THREE_A, Pi/2)
                .splineToLinearHeading(W_BD_THREE_B, Pi/2)
                .build();
        toParkFromWR = rejat.actionBuilder(W_BD_THREE_B)
                .splineToLinearHeading(PARK_CENTER, Pi/2)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        hammers.zero(),
                        gripper.close(),
                        pitchingIntake.up()
                )
        );

    }
    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal1.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
    }
    @Override
    public void start() {
        Lift lift = new Lift(hardwareMap);
        Grip gripper = new Grip(hardwareMap);
        Hammers hammers = new Hammers(hardwareMap);
        pitchingIntake pitchingIntake = new pitchingIntake(hardwareMap);

        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        if (visionPortal1.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal1.stopLiveView();
            visionPortal1.setProcessorEnabled(colourMassDetectionProcessor, false);
            visionPortal1.setProcessorEnabled(aprilTagFront, true);
        }

        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.LEFT;
        }
        switch (recordedPropPosition) {
            case LEFT:
                Actions.runBlocking(
                        toSpikeL
                );
                intake.setPower(-0.5);
                Actions.runBlocking(
                        new SequentialAction(
                                toBDFromSpikeL,
                                lift.liftUp(),
                                gripper.open()
                        )
                );
                intake.setPower(0);
                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftDown(),
                                toParkFromWL,
                                hammers.zero(),
                                pitchingIntake.down()
                        )
                );
                break;
            case MIDDLE:
                Actions.runBlocking(
                        toSpikeM
                );
                intake.setPower(-0.5);
                Actions.runBlocking(
                        new SequentialAction(
                                toBDFromSpikeM,
                                lift.liftUp(),
                                gripper.open()
                        )
                );
                intake.setPower(0);
                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftDown(),
                                toParkFromWM,
                                hammers.zero(),
                                pitchingIntake.down()
                        )
                );
                break;
            case RIGHT:
                Actions.runBlocking(
                        toSpikeR
                );
                intake.setPower(-0.5);
                Actions.runBlocking(
                        new SequentialAction(
                                toBDFromSpikeR,
                                lift.liftUp(),
                                gripper.open()
                        )
                );
                intake.setPower(0);
                Actions.runBlocking(
                        new ParallelAction(
                                lift.liftDown(),
                                toParkFromWR,
                                hammers.zero(),
                                pitchingIntake.down()
                        )
                );
                break;
        }
    }
    @Override
    public void loop() {

    }
    @Override
    public void stop() {
        colourMassDetectionProcessor.close();
        visionPortal1.close();
        visionPortal2.close();
    }
}