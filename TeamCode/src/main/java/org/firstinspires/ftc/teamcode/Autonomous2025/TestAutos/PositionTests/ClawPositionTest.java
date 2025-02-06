package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ClawPositionTest")
public class ClawPositionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(claw.testPosition());
        }

    }
}
