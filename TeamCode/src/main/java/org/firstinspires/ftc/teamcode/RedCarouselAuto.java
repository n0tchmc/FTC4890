package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class RedCarouselAuto extends LinearOpMode {
    int position = 1;
    RobotAuto robot = new RobotAuto();

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d pose = new Pose2d(-36, -62, Math.toRadians(90));
        drive.setPoseEstimate(pose);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(pose).splineTo(new Vector2d(20, -59), Math.toRadians(90)).build();
        Trajectory traj1 = drive.trajectoryBuilder(pose).strafeRight(40).build();
        Trajectory traj2 = drive.trajectoryBuilder(pose).strafeRight(40).build();
        Trajectory traj3 = drive.trajectoryBuilder(pose).strafeRight(40).build();

        robot.init(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        if (position == 1 || position == 2) {
            trajSeq = drive.trajectorySequenceBuilder(pose)
                    .forward(2)
                    .strafeLeft(20)
                    .waitSeconds(1)
                    .build();
            traj1 = drive.trajectoryBuilder(trajSeq.end()).
                    splineTo(new Vector2d(-22.5, -40), Math.toRadians(60))
                    .build();
            while (robot.clawSensor.getState()) {
                robot.claw.setPower(1);
            }
            robot.claw.setPower(0);

            robot.pivot.setPower(-0.35);
            sleep(230);
            robot.pivot.setPower(0);
            outtake(3500);
            traj2 = drive.trajectoryBuilder(traj1.end()).splineToLinearHeading(new Pose2d(-60, -35), Math.toRadians(0))
                    .build();
        }

        drive.followTrajectorySequence(trajSeq);
    }

    void outtake(int milliseconds) {
        robot.clawGrab.setPower(-1);
        sleep((milliseconds));
        robot.clawGrab.setPower(0);
    }
}
