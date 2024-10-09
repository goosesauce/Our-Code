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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        //2+5 going on outside route
                        drive.trajectorySequenceBuilder(new Pose2d(-34.75, -62.75, Math.toRadians(-90)))
                                /*.setReversed(true)
                                //.splineTo(new Vector2d(-44.75, 44.75), Math.toRadians(90))
                                .lineToLinearHeading(new Pose2d(-44.75, 44.75, Math.toRadians(270)))

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-50, 50, Math.toRadians(180)))

                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(-55, 45, Math.toRadians(195)))
                                .lineToConstantHeading(new Vector2d(-61, 37))

                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(12, 60))

                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(52.5, 33, Math.toRadians(180)))
                                //.splineTo(new Vector2d(52.5, 33), Math.toRadians(0))

                                .setReversed(false)
                                .lineToSplineHeading(new Pose2d(12, 60, Math.toRadians(180)))
                                //.splineTo(new Vector2d(12, 60), Math.toRadians(180))
                                //.splineToLinearHeading(new Pose2d(24, 60, Math.toRadians(180)), Math.toRadians(0))
                                .lineToConstantHeading(new Vector2d(-40, 60))//.lineToConstantHeading(new Vector2d(-60, -13))
                                .lineToLinearHeading(new Pose2d(-55, 45, Math.toRadians(195)))
                                .lineToConstantHeading(new Vector2d(-61, 37))

                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(-35, 60, Math.toRadians(180)))
                                //.splineTo(new Vector2d(-35, 60), Math.toRadians(0))
                                //.lineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(12, 60))
                                .lineToSplineHeading(new Pose2d(52.5, 35.5, Math.toRadians(180)))
                                //.splineTo(new Vector2d(52.5, 35.5), Math.toRadians(0))

                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(44, 60, Math.toRadians(180)))
*/

                                // 26.86 secs \/

                                //move 1
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-44.75, -44.75, Math.toRadians(-180)))
                                .lineToConstantHeading(new Vector2d(-44.75, -47))
                                .lineToLinearHeading(new Pose2d(-50, -47, Math.toRadians(135)))
                                .setReversed(false)

                                .lineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(165)))//-21, -60
                                .setReversed(true)
                                .splineTo(new Vector2d(-40, -60), Math.toRadians(0))

                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}