package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ClawPositionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);

        Actions.runBlocking(claw.testPosition());

        waitForStart();
    }
}
