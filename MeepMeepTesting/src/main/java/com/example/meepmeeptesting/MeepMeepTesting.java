package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 18)
                .followTrajectorySequence(drive ->
                        //2+5 going on outside route
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-36, -35.5, Math.toRadians(90)))
                                //place down first pixel
                                .lineToLinearHeading(new Pose2d(-55, -35.5, Math.toRadians(180)))
                                .forward(2)
                                //intake
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                                //put on backdrop
                                .waitSeconds(0.2)
                                .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-55, -35.5, Math.toRadians(180)))
                                .forward(2)
                                //intake
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                                //put on backdrop
                                .waitSeconds(0.2)
                                .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-55, -35.5, Math.toRadians(180)))
                                .forward(2)
                                //intake
                                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(35, -60, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                                //put on backdrop
                                .waitSeconds(0.2)
                                //2+6
                                .lineToLinearHeading(new Pose2d(52, -16, Math.toRadians(135)))


                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}