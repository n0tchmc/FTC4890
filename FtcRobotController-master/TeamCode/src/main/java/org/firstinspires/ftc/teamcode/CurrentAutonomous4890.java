package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "CurrentAutonomous4890", preselectTeleOp = "Teleop4890")
public class CurrentAutonomous4890 extends LinearOpMode {
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    Robot robot = new Robot();

    int ringNumber;
    boolean detection = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        //initializes the camera and sets it up for which camera will be used.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(() -> {
            phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            phoneCam.setFlashlightEnabled(true);
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);

            while (detection) {
                if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
                    ringNumber = 0;
                    detection = false;
                } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                    ringNumber = 1;
                    detection = false;
                } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                    ringNumber = 4;
                    detection = false;
                }
            }

            if (ringNumber == 0) {
                straight(1, 600);
                sleep(1000);
                armDown(700);
                robot.release(800);
                strafeLeft(1, 450);
                sleep(1000);
                straight(-1, 200);
                platformUp(100);
                robot.outtakeLeft.setPower(1);
                sleep(1000);
                outtakePush(500);
                outtakePush(500);
                outtakePush(500);
                straight(1,100);
            } else if (ringNumber == 1) {
                strafeRight(1, 150);
                straight(1, 1100);
                strafeLeft(1, 400);
                armDown(700);
                robot.release(800);
                straight(-1, 300);
                platformUp(100);
                robot.outtakeLeft.setPower(1);
                sleep(1000);
                outtakePush(250);
                outtakePush(250);
                outtakePush(250);
                straight(1,100);
            } else if (ringNumber == 4) {
                strafeRight(1, 150);
                straight(1, 2000);
                armDown(700);
                robot.release(800);
                strafeLeft(1, 300);
                straight(-1, 500);
                platformUp(100);
                robot.outtakeLeft.setPower(1);
                sleep(1000);
                outtakePush(250);
                outtakePush(250);
                outtakePush(250);
            }

            stop();
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        // An enum to define the amount of rings

        public enum RingPosition {
            FOUR, //C (box in back corner)
            ONE,  //B (box not next to wall)
            NONE  //A (box on middle line)
        }

        // Some color constants
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        // The core values which define the location and size of the sample regions
        // topleft = where the topleft point of the analysis region begins
        // region width & height determines the analysis' dimensions
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(205, 58);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 140;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        // Working variables
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystoneDeterminationPipeline.RingPosition position = SkystoneDeterminationPipeline.RingPosition.FOUR;

        // This function takes the RGB frame, converts to YCrCb,
        // and extracts the Cb channel to the 'Cb' variable
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
                position = SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }

    void straight(double power, int milliseconds) {
        robot.driveFrontLeft.setPower(-power);
        robot.driveBackLeft.setPower(-power);
        robot.driveFrontRight.setPower(-power);
        robot.driveBackRight.setPower(-power);
        sleep(milliseconds);
        robot.driveFrontLeft.setPower(0);
        robot.driveBackLeft.setPower(0);
        robot.driveFrontRight.setPower(0);
        robot.driveBackRight.setPower(0);
    }

    void strafeLeft(double power, int milliseconds) {
        robot.driveFrontRight.setPower(-power);
        robot.driveFrontLeft.setPower(power);
        robot.driveBackRight.setPower(power);
        robot.driveBackLeft.setPower(-power);
        sleep(milliseconds);
        robot.driveFrontLeft.setPower(0);
        robot.driveBackLeft.setPower(0);
        robot.driveFrontRight.setPower(0);
        robot.driveBackRight.setPower(0);
    }

    void strafeRight(double power, int milliseconds) {
        robot.driveFrontRight.setPower(power);
        robot.driveFrontLeft.setPower(-power);
        robot.driveBackRight.setPower(-power);
        robot.driveBackLeft.setPower(power);
        sleep(milliseconds);
        robot.driveFrontLeft.setPower(0);
        robot.driveBackLeft.setPower(0);
        robot.driveFrontRight.setPower(0);
        robot.driveBackRight.setPower(0);
    }

    void rotate(double power, int milliseconds) { //note: default rotate clockwise
        robot.driveFrontRight.setPower(-power);
        robot.driveFrontLeft.setPower(power);
        robot.driveBackRight.setPower(-power);
        robot.driveBackLeft.setPower(power);
        sleep(milliseconds);
        robot.driveFrontLeft.setPower(0);
        robot.driveBackLeft.setPower(0);
        robot.driveFrontRight.setPower(0);
        robot.driveBackRight.setPower(0);
    }

    void platformUp(int milliseconds) {
        robot.platformRight.setPosition(0.79);
        robot.platformLeft.setPosition(0.81);
        sleep(milliseconds);
    }

    void platformDown(int milliseconds) {
        robot.platformRight.setPosition(0.6);
        robot.platformLeft.setPosition(1);
        sleep(milliseconds);
    }

    void outtakePush(int milliseconds) {
        robot.pusher.setPosition(0.8);
        sleep(400);
        robot.pusher.setPosition(1);
        sleep(milliseconds);
    }

    void armUp(int milliseconds) {
        robot.Arm.setPower(-0.5);
        sleep(milliseconds);
        robot.Arm.setPower(0);
    }

    void armDown(int milliseconds) {
        robot.Arm.setPower(0.5);
        sleep(milliseconds);
        robot.Arm.setPower(0);
    }
}
