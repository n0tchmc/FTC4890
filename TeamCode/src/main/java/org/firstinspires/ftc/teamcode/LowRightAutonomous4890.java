package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "FAST-RIGHT-Auto4890", preselectTeleOp = "Teleop4890")
public class LowRightAutonomous4890 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline pipeline;
    Robot robot = new Robot();

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
        LEFT,
        MIDDLE,
        RIGHT,
        DETECTION_FAILED
    }


    @Override
    public void runOpMode() throws InterruptedException {

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
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        detection detectionResult = detection.DETECTION_FAILED;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose = new Pose2d(34.5, -64, Math.toRadians(90));    //initial position
        drive.setPoseEstimate(pose);

        int lowLeft = 1600 + 50;
        int lowRight = 1600 + 50;

        int midLeft = 2700 + 25;
        int midRight = 2700 + 25;

        int firstLeft = 500;
        int firstRight = 500;

        int secondLeft = 400;
        int secondRight = 400;

        int thirdLeft = 300;
        int thirdRight = 300;

        int fourthLeft = 200;
        int fourthRight = 200;

        int raiseLeft = 1350 + 50;
        int raiseRight = 1350 + 50;

        TrajectorySequence preload1 = drive.trajectorySequenceBuilder(pose)
                .forward(46)
                .addTemporalMarker(2, () -> {
                    robot.WM40.setTargetPosition(lowRight); //right
                    robot.RLM.setTargetPosition(lowLeft);  //left
                })
                .turn(Math.toRadians(-120))
                .build();

        TrajectorySequence toStack1 = drive.trajectorySequenceBuilder(preload1.end())
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(firstRight);
                    robot.RLM.setTargetPosition(firstLeft);
                })
                .turn(Math.toRadians(35))
                .forward(23)
                .build();

        TrajectorySequence fromStack1 = drive.trajectorySequenceBuilder(toStack1.end())
                .addTemporalMarker(0, () -> {
                    robot.WM40.setTargetPosition(raiseRight);
                    robot.RLM.setTargetPosition(raiseLeft);
                })
                .waitSeconds(0.3)
                .back(23)
                .addTemporalMarker(2.5, () -> {
                    robot.WM40.setTargetPosition(lowRight);
                    robot.RLM.setTargetPosition(lowLeft);
                })
                .turn(Math.toRadians(-35))
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(fromStack1.end())
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(secondRight);
                    robot.RLM.setTargetPosition(secondLeft);
                })
                .turn(Math.toRadians(35))
                .forward(23)
                .build();

        TrajectorySequence fromStack2 = drive.trajectorySequenceBuilder(toStack2.end())
                .addTemporalMarker(0, () -> {
                    robot.WM40.setTargetPosition(raiseRight);
                    robot.RLM.setTargetPosition(raiseLeft);
                })
                .waitSeconds(0.3)
                .back(23)
                .addTemporalMarker(2.5, () -> {
                    robot.WM40.setTargetPosition(lowRight);
                    robot.RLM.setTargetPosition(lowLeft);
                })
                .turn(Math.toRadians(-35))
                .build();

        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(fromStack2.end())
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(thirdRight);
                    robot.RLM.setTargetPosition(thirdLeft);
                })
                .turn(Math.toRadians(35))
                .forward(23)
                .build();

        TrajectorySequence fromStack3 = drive.trajectorySequenceBuilder(toStack3.end())
                .addTemporalMarker(0, () -> {
                    robot.WM40.setTargetPosition(raiseRight);
                    robot.RLM.setTargetPosition(raiseLeft);
                })
                .waitSeconds(0.3)
                .back(23)
                .addTemporalMarker(2.5, () -> {
                    robot.WM40.setTargetPosition(lowRight);
                    robot.RLM.setTargetPosition(lowLeft);
                })
                .turn(Math.toRadians(-35))
                .build();

        TrajectorySequence toStack4 = drive.trajectorySequenceBuilder(fromStack3.end())
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(thirdRight);
                    robot.RLM.setTargetPosition(thirdLeft);
                })
                .turn(Math.toRadians(35))
                .forward(23)
                .build();

        TrajectorySequence fromStack4 = drive.trajectorySequenceBuilder(toStack4.end())
                .addTemporalMarker(0, () -> {
                    robot.WM40.setTargetPosition(raiseRight);
                    robot.RLM.setTargetPosition(raiseLeft);
                })
                .waitSeconds(0.3)
                .back(23)
                .addTemporalMarker(2.5, () -> {
                    robot.WM40.setTargetPosition(lowRight);
                    robot.RLM.setTargetPosition(lowLeft);
                })
                .turn(Math.toRadians(-35))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(fromStack4.end())
                .back(4.7)
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(0);
                    robot.RLM.setTargetPosition(0);
                })
                .turn(Math.toRadians(45))
                .back(23.5)
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(fromStack4.end())
                .back(3)
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(0);
                    robot.RLM.setTargetPosition(0);
                })
                .turn(Math.toRadians(35))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(fromStack4.end())
                .back(4.7)
                .addTemporalMarker(0.2, () -> {
                    robot.WM40.setTargetPosition(0);
                    robot.RLM.setTargetPosition(0);
                })
                .turn(Math.toRadians(45))
                .forward(24)
                .build();

        while (!isStarted()) {
            ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
                if (detectionResult == detection.LEFT) {
                    telemetry.addLine("Plan: Left Side");
                } else if (detectionResult == detection.RIGHT) {
                    telemetry.addLine("Plan: Right Side");
                } else {
                    telemetry.addLine("Plan: Middle Side");
                }

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

                    if (detections.get(0).id == 1) {
                        detectionResult = detection.LEFT;
                    } else if (detections.get(0).id == 2) {
                        detectionResult = detection.MIDDLE;
                    } else if (detections.get(0).id == 3) {
                        detectionResult = detection.RIGHT;
                    }

                    for (AprilTagDetection detection : detections) {
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    }
                    if (detectionResult == detection.LEFT) {
                        telemetry.addLine("Left Side");
                    } else if (detectionResult == detection.MIDDLE) {
                        telemetry.addLine("Middle Side");
                    } else if (detectionResult == detection.RIGHT) {
                        telemetry.addLine("Right Side");
                    }
                }

                telemetry.update();
            }
        }

        robot.RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.WM40.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.WM40.setPower(1);
        robot.RLM.setPower(1);
        robot.RLM.setTargetPosition(0);
        robot.WM40.setTargetPosition(0);
        robot.RLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WM40.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        if (detectionResult == detection.LEFT) {
            //first preload cone
            robot.claw.setPosition(0);
            sleep(300);
            drive.followTrajectorySequence(preload1);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //first cone in stack
            drive.followTrajectorySequence(toStack1);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack1);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //second cone in stack
            drive.followTrajectorySequence(toStack2);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack2);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //third cone in stack
            drive.followTrajectorySequence(toStack3);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack3);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //fourth cone in stack
            drive.followTrajectorySequence(toStack4);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack4);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            drive.followTrajectorySequence(park1);

        } else if (detectionResult == detection.RIGHT) {
            //first preload cone
            robot.claw.setPosition(0);
            sleep(300);
            drive.followTrajectorySequence(preload1);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //first cone in stack
            drive.followTrajectorySequence(toStack1);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack1);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //second cone in stack
            drive.followTrajectorySequence(toStack2);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack2);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //third cone in stack
            drive.followTrajectorySequence(toStack3);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack3);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //fourth cone in stack
            drive.followTrajectorySequence(toStack4);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack4);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            drive.followTrajectorySequence(park3);


        } else {    //Middle
            //first preload cone
            robot.claw.setPosition(0);
            sleep(300);
            drive.followTrajectorySequence(preload1);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //first cone in stack
            drive.followTrajectorySequence(toStack1);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack1);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //second cone in stack
            drive.followTrajectorySequence(toStack2);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack2);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //third cone in stack
            drive.followTrajectorySequence(toStack3);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack3);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            //fourth cone in stack
            drive.followTrajectorySequence(toStack4);
            robot.claw.setPosition(0);
            sleep(250);
            drive.followTrajectorySequence(fromStack4);
            sleep(100);
            robot.claw.setPosition(0.4);
            sleep(100);

            drive.followTrajectorySequence(park2);
        }

        if(isStopRequested()) return;
    }
}