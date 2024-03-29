package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 45, 6.281111240386963, Math.toRadians(60), 13.26)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34.5, -64, Math.toRadians(90)))
                                .waitSeconds(0.4)
                                .forward(51)
                                .turn(Math.toRadians(50))

                                .waitSeconds(2.3)
                                .forward(3.7)
                                .waitSeconds(0.3)
                                .back(3.7)
                                .waitSeconds(2.3)

                                .turn(Math.toRadians(-140))
                                .forward(26)
                                .waitSeconds(2.3)
                                .back(26)
                                .turn(Math.toRadians(140))

                                .waitSeconds(2.3)
                                .forward(3.7)
                                .waitSeconds(0.3)
                                .back(3.7)
                                .waitSeconds(2.3)

                                .turn(Math.toRadians(-140))
                                .forward(26)
                                .waitSeconds(2.3)
                                .back(26)
                                .turn(Math.toRadians(140))

                                .waitSeconds(2.3)
                                .forward(3.7)
                                .waitSeconds(0.3)
                                .back(3.7)
                                .waitSeconds(2.3)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}