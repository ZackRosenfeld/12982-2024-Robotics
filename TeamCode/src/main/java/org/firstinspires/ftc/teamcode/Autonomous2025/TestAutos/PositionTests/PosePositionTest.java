package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "PosePositionTest")
public class PosePositionTest extends LinearOpMode {

    public static double POSITION_X = 0;
    public static double POSITION_Y = 0;
    public static double HEADING = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, Math.PI / 2));

        TrajectoryActionBuilder driveToPosition = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(POSITION_X, POSITION_Y), HEADING);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(new ParallelAction(driveToPosition.build(),
                    lift.testPosition(),
                    arm.testPosition(),
                    wrist.testPosition(),
                    claw.testPosition()));
        }
    }
}
