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


        Actions.runBlocking(claw.startClaw());

        waitForStart();
    }
}
