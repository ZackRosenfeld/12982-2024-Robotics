package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "DrivetrainPositionTest")
public class DrivetrainPositionTest extends LinearOpMode {

    public static double POSITION_X = 0;
    public static double POSITION_Y = 0;
    public static double HEADING = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, Math.PI / 2));

        TrajectoryActionBuilder driveToPosition = drive.actionBuilder(new Pose2d(0, 0, Math.PI / 2))
                .strafeToLinearHeading(new Vector2d(POSITION_X, POSITION_Y), HEADING);

        waitForStart();


        Actions.runBlocking(driveToPosition.build());

    }
}
