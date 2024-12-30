package org.firstinspires.ftc.teamcode.Autonomous2025;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "SpecimenPickupTest", group = "Test_Autos")
public class SpecimenPickupTest extends LinearOpMode {

    public static double FORWARD_INITIAL = 6;
    public static double BACKWARD_PICKUP = 5;
    public static boolean SCORE_SPECIMEN = false;
    public static double FORWARD_AFTER_PICKUP = 31;
    public static double WAIT_SECONDS = 2;
    public static double SCORING_POSITION = 36;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI / 2));
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder pickupMove = drive.actionBuilder(new Pose2d(0, 0, Math.PI / 2))
                .lineToY(FORWARD_INITIAL)
                .lineToY(BACKWARD_PICKUP);

        TrajectoryActionBuilder scoreMove = drive.actionBuilder(new Pose2d(0, BACKWARD_PICKUP, Math.PI / 2))
                .lineToY(FORWARD_AFTER_PICKUP)
                .waitSeconds(WAIT_SECONDS)
                .lineToY(SCORING_POSITION);


        Actions.runBlocking(claw.startClaw());

        waitForStart();

        Actions.runBlocking(new SequentialAction(pickupMove.build(), claw.closeClaw()));

        if (SCORE_SPECIMEN) {
            Actions.runBlocking(new SequentialAction( new ParallelAction(lift.aboveHighBar(),
                            scoreMove.build()),
                    lift.scoringHighBar()));
        }
    }
}
