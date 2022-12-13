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
                .setConstraints(30, 45, 6.281111240386963, Math.toRadians(60), 13.26)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34, -63, Math.toRadians(90)))
                                //LOW FIRST
                                .forward(5)
                                .turn(Math.toRadians(45))
                                .addDisplacementMarker(() -> {
                                    //arm raise
                                })
                                .waitSeconds(1)
                                .forward(2)
                                .addDisplacementMarker(() -> {
                                    //claw drop
                                })
                                .waitSeconds(0.3)
                                .back(3)
                                .addDisplacementMarker(() -> {
                                    //arm lower
                                })
                                //GO TO STACK
                                .turn(Math.toRadians(-45))
                                .forward(46)
                                .turn(Math.toRadians(-90))
                                .forward(25)
                                .addDisplacementMarker(() -> {
                                    //grab then raise
                                })
                                .waitSeconds(0.6)
                                .back(25)
                                //HIGH GOAL 1
                                .turn(Math.toRadians(135))
                                .forward(3)
                                .addDisplacementMarker(() -> {
                                    //raise
                                })
                                .waitSeconds(1) //wait for arm raise
                                .forward(1)
                                .addDisplacementMarker(() -> {
                                    //release
                                })
                                .waitSeconds(0.3)
                                .back(4)
                                .addDisplacementMarker(() -> {
                                    //lower
                                })
                                .turn(Math.toRadians(-135))
                                //BACK TO STACK
                                .forward(25)
                                .addDisplacementMarker(() -> {
                                    //grab then raise
                                })
                                .waitSeconds(0.6)
                                .back(25)
                                //HIGH GOAL 2
                                .turn(Math.toRadians(135))
                                .forward(3)
                                .addDisplacementMarker(() -> {
                                    //raise
                                })
                                .waitSeconds(1) //wait for arm raise
                                .forward(1)
                                .addDisplacementMarker(() -> {
                                    //release
                                })
                                .waitSeconds(0.3)
                                .back(4)
                                .addDisplacementMarker(() -> {
                                    //lower
                                })
                                .turn(Math.toRadians(-135))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}