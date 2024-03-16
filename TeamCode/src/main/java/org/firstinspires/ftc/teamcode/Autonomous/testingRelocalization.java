package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
@TeleOp(name = "BackboardNewRedAuton")
public class testingRelocalization extends CommandOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    int region = 0;
    double[] positions;
    Action traj1, traj2, traj3;
    public boolean running;

    @Override
    public void initialize() {
        initAprilTag();



        while (opModeInInit()) {

        }

    }
    @Override
    public void run() {
        positions = getPosition(9);
        telemetry.addData("PosX", -positions[0]);
        telemetry.addData("PosY", positions[1]);
        telemetry.addData("PosYaw", positions[2]);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-40, -60, Math.toRadians(90)));

        drive.pose = new Pose2d(-40, -60, Math.toRadians(90));

        traj1 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-45, -15, Math.toRadians(270)), Math.PI/2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -21, Math.toRadians(270)), Math.PI)
                .waitSeconds(1)
                .build();
        traj2 = drive.actionBuilder(new Pose2d(-48, -21, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-51, -11.5, Math.toRadians(180)), Math.toRadians(270))
                .build();
        traj3 = drive.actionBuilder(new Pose2d(-51 - (cameradistancefromstackfromtestingatpractice + positions[0]), -11.5 + positions[1], Math.toRadians(180 + positions[2])))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-60, -11.5, Math.toRadians(180)), Math.toRadians(270))
                .build();

        if(running) {
            Actions.runBlocking(
                    traj1,
                    traj2,
                    traj3,
                    running = false
            );
        }
        telemetry.update();
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                .build();

        aprilTag.setDecimation(3);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        builder.setAutoStopLiveView(false);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();

    }

    public double[] getPosition(int id) {
        double[] stoof = new double[3];
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.id==id) {
                stoof[0] = detection.ftcPose.x;
                stoof[1] = detection.ftcPose.y;
                stoof[2] = detection.ftcPose.yaw;
            }
        }
        return stoof;
    }
}