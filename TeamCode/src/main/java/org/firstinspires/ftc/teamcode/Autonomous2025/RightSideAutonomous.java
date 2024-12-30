package org.firstinspires.ftc.teamcode.Autonomous2025;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import kotlin.jvm.internal.TypeParameterReference;

@Config
@Autonomous(name = "rightAutonomous2025")
public class RightSideAutonomous extends LinearOpMode {

    public static double WAIT_SECONDS = 2; // Amount of time waited for human player in seconds

    @Override
    public void runOpMode() throws InterruptedException {
        // hardware
        Pose2d initialPose = new Pose2d(79.625, 6.95, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // Drive Trajectories
        TrajectoryActionBuilder pickupSpecimen = drive.actionBuilder(initialPose).
                lineToY(11).
                lineToY(9.3);

        TrajectoryActionBuilder toSubmersibleInitial = drive.actionBuilder(new Pose2d(77.625, 9.3, Math.PI / 2))
                .lineToY(31);

        TrajectoryActionBuilder submersibleScoring = drive.actionBuilder(new Pose2d(77.625, 31, Math.PI / 2))
                .lineToY(36);

        TrajectoryActionBuilder strafeToObservasionZone = drive.actionBuilder(new Pose2d(77.625, 36, Math.PI / 2))
                .strafeToLinearHeading(new Vector2d(87.5, 26), 0)
                .strafeToLinearHeading(new Vector2d (133, 29), -Math.PI / 2);

        TrajectoryActionBuilder forwardToSpecimen = drive.actionBuilder(new Pose2d(133, 29, -Math.PI / 2))
                .waitSeconds(WAIT_SECONDS)
                .lineToY(21.2);

        TrajectoryActionBuilder backToSubmersible = drive.actionBuilder(new Pose2d(133, 21.2, -Math.PI / 2))
                .splineToLinearHeading(new Pose2d(103, 28, 0), Math.PI)
                .splineToLinearHeading(new Pose2d(73, 38, Math.PI / 2), Math.PI / 2);

        Actions.runBlocking(claw.startClaw());

        waitForStart();

        Actions.runBlocking(new SequentialAction(pickupSpecimen.build(),
                claw.closeClaw(),
                new ParallelAction(toSubmersibleInitial.build(),
                        lift.aboveHighBar()),
                submersibleScoring.build(),
                lift.scoringHighBar(),
                new ParallelAction(strafeToObservasionZone.build(), lift.down(), claw.openClaw()),
                forwardToSpecimen.build(),
                claw.closeClaw(),
                new ParallelAction(backToSubmersible.build(),
                        lift.aboveHighBar()),
                lift.scoringHighBar()));
    }
}
