package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // Stored Positions
    // Each pose is stored in its own class for easier tuning
    public static class PoseInitial {
        public double POSITION_X = 10;
        public double POSITION_Y = -63;
        public double HEADING = Math.PI / 2;
    }

    private static PoseInitial poseInitial = new PoseInitial();

    public static class ScoringSpecimen {
        public double POSITION_X = 3;
        public double POSITION_Y = -40;
        public double HEADING = Math.PI / 2;
    }

    private static ScoringSpecimen scoringSpecimen = new ScoringSpecimen();

    public static class WallPickup {
        public double POSITION_X = 48;
        public double POSITION_Y = -58;
        public double HEADING = Math.PI / 2;
    }
    private static WallPickup wallPickup = new WallPickup();

    public static class Sample1PrePush {
        public double POSITION_X = 43;
        public double POSITION_Y = -10;
        public double HEADING = Math.PI / 2;
    }
    private static Sample1PrePush sample1PrePush = new Sample1PrePush();

    public static class Sample1PostPush {
        public double POSITION_X = 43;
        public double POSITION_Y = -55;
        public double HEADING = Math.PI / 2;
    }
    private static Sample1PostPush sample1PostPush = new Sample1PostPush();

    public static class Sample2PrePush {
        public double POSITION_X = 53;
        public double POSITION_Y = -10;
        public double HEADING = Math.PI / 2;
    }
    private static Sample2PrePush sample2PrePush = new Sample2PrePush();

    public static class Sample2PostPush {
        public double POSITION_X = 53;
        public double POSITION_Y = -55;
        public double HEADING = Math.PI / 2;
    }
    private static Sample2PostPush sample2PostPush = new Sample2PostPush();

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        /*myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(30)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());*/

        TrajectoryActionBuilder scoreSpecimen1 = myBot.getDrive().actionBuilder(new Pose2d(poseInitial.POSITION_X, poseInitial.POSITION_Y, poseInitial.HEADING))
                .strafeToConstantHeading(new Vector2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y));

        TrajectoryActionBuilder pushSamples = myBot.getDrive().actionBuilder(new Pose2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y, scoringSpecimen.HEADING))
                .setTangent(0)
                .lineToXConstantHeading(20)
                .splineToConstantHeading(new Vector2d(37, -15), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(sample1PrePush.POSITION_X, sample1PrePush.POSITION_Y), 0)
                .strafeToConstantHeading(new Vector2d(sample1PostPush.POSITION_X, sample1PostPush.POSITION_Y))
                .lineToYConstantHeading(-30)
                .splineToConstantHeading(new Vector2d(sample2PrePush.POSITION_X, sample2PrePush.POSITION_Y), 0)
                .strafeToConstantHeading(new Vector2d(sample2PostPush.POSITION_X, sample2PostPush.POSITION_Y));

        TrajectoryActionBuilder pickupSpecimen2 = myBot.getDrive().actionBuilder(new Pose2d(sample2PostPush.POSITION_X, sample2PostPush.POSITION_Y, sample2PostPush.HEADING))
                        .strafeToConstantHeading(new Vector2d(wallPickup.POSITION_X, wallPickup.POSITION_Y + 20))
                        .waitSeconds(1)
                        .lineToY(wallPickup.POSITION_Y);

        TrajectoryActionBuilder scoreSpecimen2 = myBot.getDrive().actionBuilder(new Pose2d(wallPickup.POSITION_X, wallPickup.POSITION_Y, wallPickup.HEADING))
                        .strafeToConstantHeading(new Vector2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y));

        TrajectoryActionBuilder pickupSpecimen3 = myBot.getDrive().actionBuilder(new Pose2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y, scoringSpecimen.HEADING))
                .strafeToConstantHeading(new Vector2d(wallPickup.POSITION_X, wallPickup.POSITION_Y + 20))
                .waitSeconds(1)
                .lineToY(wallPickup.POSITION_Y);

        TrajectoryActionBuilder scoreSpecimen3 = myBot.getDrive().actionBuilder(new Pose2d(wallPickup.POSITION_X, wallPickup.POSITION_Y, wallPickup.HEADING))
                .strafeToConstantHeading(new Vector2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y));

        TrajectoryActionBuilder park = myBot.getDrive().actionBuilder(new Pose2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y, scoringSpecimen.HEADING))
                .strafeToConstantHeading(new Vector2d(wallPickup.POSITION_X, wallPickup.POSITION_Y));



        myBot.runAction(new SequentialAction(scoreSpecimen1.build(),
                pushSamples.build(),
                pickupSpecimen2.build(),
                scoreSpecimen2.build(),
                pickupSpecimen3.build(),
                scoreSpecimen3.build(),
                park.build()));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}