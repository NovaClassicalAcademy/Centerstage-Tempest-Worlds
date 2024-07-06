package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.BD_BD_ONE_OFF;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.BD_BD_THREE_OFF;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.BD_BD_TWO_OFF;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.BD_SPIKE_ONE;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.BD_SPIKE_THREE;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.BD_SPIKE_TWO;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.PARK_CORNER;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.ensuredDropL;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.ensuredDropM;
import static org.firstinspires.ftc.teamcode.AutoConstants.Blue.ensuredDropR;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColourMassDetectionProcessor;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous
public class redW0 extends OpMode {
    public VisionPortal visionPortal1, visionPortal2;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
    public static double Pi = Math.PI;
    Action toSpikeL, toBDFromSpikeL, toSpikeM, toBDFromSpikeM, toSpikeR, toBDFromSpikeR, toParkFromBDL, toParkFromBDM, toParkFromBDR, dropEnsuredL, dropEnsuredM, dropEnsuredR;

    public class Grip {
        private Servo gripper;

        public Grip(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "gripper");
        }
        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.45);
                try {
                    Thread.sleep(600);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
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
                drop1.setPosition(0.4);
                drop2.setPosition(0.6);
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
                hammerL.setPosition(0.6);
                hammerR.setPosition(0.4);
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

            liftR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    axonR.setPosition(0.13);
                    axonL.setPosition(0.41);
                    liftL.setPower(-0.6);
                    liftR.setPower(-0.6);
                    initialized = true;
                }

                double pos = -liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1000) {
                    return true;
                } else {
                    liftR.setPower(0);
                    liftL.setPower(0);
                    axonR.setPosition(0.40);
                    axonL.setPosition(0.38);
                    twist.setPosition(0.50);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class OuttakeIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                axonR.setPosition(0.13);
                axonL.setPosition(0.42);
                twist.setPosition(0.5);
                return false;
            }
        }
        public Action outtakeIn() {
            return new OuttakeIn();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(0.3);
                    initialized = true;
                }

                double pos = -liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 80) {
                    return true;
                } else {
                    liftL.setPower(0);
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

        Scalar lower = new Scalar(0, 50, 50); // the lower hsv threshold for RED
        Scalar upper = new Scalar(125, 255, 255); // the upper hsv threshold for RED

        double minArea = 200;

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
        );

        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colourMassDetectionProcessor)
                .enableLiveView(true)
                .build();

        MecanumDrive rejat = new MecanumDrive(hardwareMap, new Pose2d(18.50, 63, Math.toRadians(-90)));

        //BD Blue LEFT 2+0 Auto (Park in Corner)
        toSpikeL = rejat.actionBuilder(rejat.pose)
                .splineToLinearHeading(BD_SPIKE_ONE, 0)
                .build();
        toBDFromSpikeL = rejat.actionBuilder(BD_SPIKE_ONE)
                .setReversed(true)
                .splineToLinearHeading(BD_BD_ONE_OFF, 0)
                .build();
        dropEnsuredL = rejat.actionBuilder(BD_BD_ONE_OFF)
                .setReversed(false)
                .splineToLinearHeading(ensuredDropL, 0)
                .build();
        toParkFromBDL = rejat.actionBuilder(ensuredDropL)
                .setReversed(true)
                .splineToLinearHeading(PARK_CORNER, 0)
                .build();
        //BD Blue Middle 2+0 Auto (Park in Corner)
        toSpikeM = rejat.actionBuilder(rejat.pose)
                .splineToLinearHeading(BD_SPIKE_TWO, 0)
                .build();
        toBDFromSpikeM = rejat.actionBuilder(BD_SPIKE_TWO)
                .splineToLinearHeading(BD_BD_TWO_OFF, 0)
                .build();
        dropEnsuredM = rejat.actionBuilder(BD_BD_TWO_OFF)
                .splineToLinearHeading(ensuredDropM, 0)
                .build();
        toParkFromBDM = rejat.actionBuilder(ensuredDropM)
                .splineToLinearHeading(PARK_CORNER, 0)
                .build();
        //BD Blue Right 2+0 Auto (Park in Corner)
        toSpikeR = rejat.actionBuilder(rejat.pose)
                .splineToLinearHeading(BD_SPIKE_THREE, 0)
                .build();
        toBDFromSpikeR = rejat.actionBuilder(BD_SPIKE_THREE)
                .setReversed(true)
                .splineToLinearHeading(BD_BD_THREE_OFF, 0)
                .build();
        dropEnsuredR = rejat.actionBuilder(BD_BD_THREE_OFF)
                .setReversed(false)
                .splineToLinearHeading(ensuredDropR, 0)
                .build();
        toParkFromBDR = rejat.actionBuilder(ensuredDropR)
                .setReversed(true)
                .splineToLinearHeading(PARK_CORNER, Pi)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        hammers.intake(),
                        gripper.close()
                )
        );
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal1.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
        telemetry.update();
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
        }

        if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.LEFT;
        }
        switch (recordedPropPosition) {
            case LEFT:
                Actions.runBlocking(
                        new SequentialAction(
                                toSpikeL,
                                hammers.zero()
                        )
                );
                break;
            case MIDDLE:
                Actions.runBlocking(
                        new SequentialAction(
                                toSpikeM,
                                hammers.zero()
                        )
                );

                break;
            case RIGHT:
                Actions.runBlocking(
                        new SequentialAction(
                                toSpikeR,
                                hammers.zero()
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