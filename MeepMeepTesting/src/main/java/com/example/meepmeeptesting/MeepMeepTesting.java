package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 155)
                .build();

        Pose2d initialPose = new Pose2d(-36, -(60+((double) (24-17)/2)), Math.toRadians(90.0));

        TrajectoryActionBuilder step1 = myBot.getDrive()
                .actionBuilder(initialPose)
                .strafeTo(new Vector2d(-48.0, -48.0))
                .turnTo(Math.toRadians(-135.0))
                .waitSeconds(3)

                .strafeTo(new Vector2d(-36.0, -24.0-2.25))
                .turnTo(Math.toRadians(180.0))
                .waitSeconds(3)

                .strafeTo(new Vector2d(-48, -48))
                .turnTo(Math.toRadians(-135.0))
                .waitSeconds(3)

                .strafeTo(new Vector2d(-34, 0));


        myBot.runAction(step1.build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}