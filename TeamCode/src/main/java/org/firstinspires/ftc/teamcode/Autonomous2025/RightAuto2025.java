package org.firstinspires.ftc.teamcode.Autonomous2025;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Util;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RightAuto2025 extends LinearOpMode {

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

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(poseInitial.POSITION_X, poseInitial.POSITION_Y, poseInitial.HEADING));
        Util util = new Util();

        TrajectoryActionBuilder scoreSpecimen1 = drive.actionBuilder(new Pose2d(poseInitial.POSITION_X, poseInitial.POSITION_Y, poseInitial.HEADING))
                .strafeToConstantHeading(new Vector2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y));

        TrajectoryActionBuilder pushSamples = drive.actionBuilder(new Pose2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y, scoringSpecimen.HEADING))
                .setTangent(0)
                .lineToXConstantHeading(20)
                .splineToConstantHeading(new Vector2d(37, -15), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(sample1PrePush.POSITION_X, sample1PrePush.POSITION_Y), 0)
                .strafeToConstantHeading(new Vector2d(sample1PostPush.POSITION_X, sample1PostPush.POSITION_Y))
                .lineToYConstantHeading(-30)
                .splineToConstantHeading(new Vector2d(sample2PrePush.POSITION_X, sample2PrePush.POSITION_Y), 0)
                .strafeToConstantHeading(new Vector2d(sample2PostPush.POSITION_X, sample2PostPush.POSITION_Y));

        TrajectoryActionBuilder pickupSpecimen2 = drive.actionBuilder(new Pose2d(sample2PostPush.POSITION_X, sample2PostPush.POSITION_Y, sample2PostPush.HEADING))
                .strafeToConstantHeading(new Vector2d(wallPickup.POSITION_X, wallPickup.POSITION_Y + 20))
                .waitSeconds(1)
                .lineToY(wallPickup.POSITION_Y);

        TrajectoryActionBuilder scoreSpecimen2 = drive.actionBuilder(new Pose2d(wallPickup.POSITION_X, wallPickup.POSITION_Y, wallPickup.HEADING))
                .strafeToConstantHeading(new Vector2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y));

        TrajectoryActionBuilder pickupSpecimen3 = drive.actionBuilder(new Pose2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y, scoringSpecimen.HEADING))
                .strafeToConstantHeading(new Vector2d(wallPickup.POSITION_X, wallPickup.POSITION_Y + 20))
                .waitSeconds(1)
                .lineToY(wallPickup.POSITION_Y);

        TrajectoryActionBuilder scoreSpecimen3 = drive.actionBuilder(new Pose2d(wallPickup.POSITION_X, wallPickup.POSITION_Y, wallPickup.HEADING))
                .strafeToConstantHeading(new Vector2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(scoringSpecimen.POSITION_X, scoringSpecimen.POSITION_Y, scoringSpecimen.HEADING))
                .strafeToConstantHeading(new Vector2d(wallPickup.POSITION_X, wallPickup.POSITION_Y));


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(scoreSpecimen1.build(),
                    arm.scoringHighBar(),
                    wrist.wristDown()),
                wrist.wristUp(),
                util.pause(1),
                claw.openClaw(),
                new ParallelAction(pushSamples.build(),
                        arm.home(),
                        wrist.wristDown()),
                new ParallelAction(pickupSpecimen2.build(),
                        arm.wallPickup(),
                        wrist.wallPickup(),
                        claw.openClaw()),
                claw.closeClaw(),
                util.pause(.5),
                new ParallelAction(scoreSpecimen2.build(),
                        arm.scoringHighBar(),
                        wrist.wristDown()),
                wrist.wristUp(),
                util.pause(1),
                claw.openClaw(),
                new ParallelAction(pickupSpecimen3.build(),
                        arm.wallPickup(),
                        wrist.wallPickup()),
                claw.closeClaw(),
                util.pause(.5),
                new ParallelAction(scoreSpecimen3.build(),
                        arm.scoringHighBar(),
                        wrist.wristDown()),
                wrist.wristUp(),
                util.pause(1),
                claw.openClaw(),
                new ParallelAction(park.build(),
                        arm.home(),
                        wrist.wristDown())));
    }
}
