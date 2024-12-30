package org.firstinspires.ftc.teamcode.Autonomous2025;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BlueRightPathTest", group = "Test_Autos")
public class BlueRightTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(79.625, 6.95, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Claw claw = new Claw(hardwareMap);

        TrajectoryActionBuilder forward = drive.actionBuilder(initialPose)
                .lineToY(36);

        TrajectoryActionBuilder rightBackward = drive.actionBuilder(new Pose2d(79.625, 36, Math.PI / 2))
                .setTangent(-Math.PI / 6)
                .lineToXConstantHeading(94.5);

        TrajectoryActionBuilder forwardRightSpline = drive.actionBuilder(new Pose2d(94.5, 27.412, Math.PI / 2))
                .splineToLinearHeading(new Pose2d(115, 47, 0), Math.PI / 2)
                .splineToLinearHeading(new Pose2d(130, 60, -Math.PI / 2), 0);

        Actions.runBlocking(claw.startClaw());

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(forward.build(), rightBackward.build(), claw.closeClaw(), claw.openClaw(), forwardRightSpline.build()));

    }
}
