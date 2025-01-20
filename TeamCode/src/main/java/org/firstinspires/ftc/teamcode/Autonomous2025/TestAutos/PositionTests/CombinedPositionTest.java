package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CombinedPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        Actions.runBlocking(new ParallelAction(lift.testPosition(),
                arm.testPosition(),
                wrist.testPosition(),
                claw.testPosition()));

        waitForStart();
    }
}
