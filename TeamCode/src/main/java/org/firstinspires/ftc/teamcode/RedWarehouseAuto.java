package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(preselectTeleOp = "Teleop4890")
public class RedWarehouseAuto extends LinearOpMode {
    RobotAuto robot = new RobotAuto();
    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 webcam at 1280x720.
    double fx = 1430;
    double fy = 1430;
    double cx = 480;
    double cy = 620;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public enum detection {
        LEVEL_ONE,
        LEVEL_TWO,
        LEVEL_THREE,
        DETECTION_FAILED
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        //initializes the camera and sets it up for which camera will be used.
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        detection detectionResult = detection.DETECTION_FAILED;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d pose = new Pose2d(12, -62, Math.toRadians(90));
        drive.setPoseEstimate(pose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(pose).strafeRight(40).build();
        TrajectorySequence defaultTraj = drive.trajectorySequenceBuilder(pose)
                .splineTo(new Vector2d(1.5, -42), Math.toRadians(125))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    while (robot.clawSensor.getState()) {
                        robot.claw.setPower(1);
                    }
                    robot.claw.setPower(0);
                    robot.clawGrab.setPower(-1);
                })
                .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                    robot.clawGrab.setPower(0);
                })
                .waitSeconds(5)
                .splineToLinearHeading(new Pose2d(12, -64), Math.toRadians(0))
                .forward(40)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(pose).strafeRight(40).build();
        Trajectory traj2 = drive.trajectoryBuilder(pose).strafeRight(40).build();
        Trajectory traj3 = drive.trajectoryBuilder(pose).strafeRight(40).build();

        while (!isStarted()) {
            ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if (detections.size() == 0) {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                        pipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                        pipeline.setDecimation(DECIMATION_HIGH);
                    }

                    if ((detections.get(0).pose.x * FEET_PER_METER) < 0) {
                        detectionResult = detection.LEVEL_ONE;
                    } else if ((detections.get(0).pose.x * FEET_PER_METER) < 2 && (detections.get(0).pose.x * FEET_PER_METER) > 0) {
                        detectionResult = detection.LEVEL_TWO;
                    } else if ((detections.get(0).pose.x * FEET_PER_METER) > 2) {
                        detectionResult = detection.LEVEL_THREE;
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                    }
                }

                telemetry.update();
            }
        }

        waitForStart();

        if (detectionResult == detection.LEVEL_ONE) {
            trajSeq = drive.trajectorySequenceBuilder(pose)
                    .splineTo(new Vector2d(1.5, -42), Math.toRadians(130))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        while (robot.clawSensor.getState()) {
                            robot.claw.setPower(1);
                        }
                        robot.claw.setPower(0);

                        robot.pivot.setPower(-0.35);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.pivot.setPower(0);
                        robot.clawGrab.setPower(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                        robot.clawGrab.setPower(0);
                    })
                    .waitSeconds(4)
                    .splineToLinearHeading(new Pose2d(12, -64), Math.toRadians(0))
                    .forward(40)
                    .build();
        } else if (detectionResult == detection.LEVEL_TWO) {
            trajSeq = drive.trajectorySequenceBuilder(pose)
                    .splineTo(new Vector2d(1.5, -42), Math.toRadians(125))
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        while (robot.clawSensor.getState()) {
                            robot.claw.setPower(1);
                        }
                        robot.claw.setPower(0);
                        robot.clawGrab.setPower(-1);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(5, () -> {
                        robot.clawGrab.setPower(0);
                    })
                    .waitSeconds(4)
                    .splineToLinearHeading(new Pose2d(12, -64), Math.toRadians(0))
                    .forward(40)
                    .build();
        } else if (detectionResult == detection.LEVEL_THREE) {
            traj1 = drive.trajectoryBuilder(pose).splineTo(new Vector2d(4.5, -46), Math.toRadians(120)).build();
            traj2 = drive.trajectoryBuilder(traj1.end()).splineToLinearHeading(new Pose2d(12, -64), Math.toRadians(0)).build();
            traj3 = drive.trajectoryBuilder(traj2.end()).forward(40).build();
        }

        if(isStopRequested()) return;

        if (detectionResult == detection.DETECTION_FAILED) {
            drive.followTrajectorySequence(defaultTraj);
        } else if (detectionResult != detection.LEVEL_THREE) {
                drive.followTrajectorySequence(trajSeq);
        }  else {
                drive.followTrajectory(traj1);
                while (robot.clawSensor.getState()) {
                    robot.claw.setPower(1);
                }
                robot.claw.setPower(0);

                while (robot.armSensor.getState()) {
                    robot.arm.setPower(0.9);
                }
                robot.arm.setPower(0);

                outtake(3000);
                robot.arm.setPower(-0.9);
                sleep(1000);
                robot.arm.setPower(0);
                drive.followTrajectory(traj2);
                drive.followTrajectory(traj3);
        }
    }

    void outtake(int milliseconds) {
        robot.clawGrab.setPower(-1);
        sleep((milliseconds));
        robot.clawGrab.setPower(0);
    }
}

