package com.example.android.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.entity.TrajectorySequenceEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        DriveShim driveSim = new DriveShim(DriveTrainType.MECANUM, new Constraints(56.9087263114899325, 56.9087263114899325, Math.toRadians(180), Math.toRadians(180), 13.45), new Pose2d(0, 0));

        TrajectorySequence redWarehoue = driveSim.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(2.5, -43), Math.toRadians(125))
                .addDisplacementMarker(() -> {
                })
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(12, -62), Math.toRadians(0))
                .forward(40)
                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(14.5, 18)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(56.9087263114899325, 30, Math.toRadians(180), Math.toRadians(180), 13.45)
                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
//                                .splineToConstantHeading(new Vector2d(-59.75, -60), Math.toRadians(90))
//                                .waitSeconds(0.5)
//                                .lineTo(new Vector2d(-59.75, -61.5))
//                                .waitSeconds(1)adb
//                                .splineTo(new Vector2d(-26.5, -43), Math.toRadians(55))
//                                .waitSeconds(1)
//                                .splineToLinearHeading(new Pose2d(-63, -36), Math.toRadians(0))
//                                .build()
                        redWarehoue
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
