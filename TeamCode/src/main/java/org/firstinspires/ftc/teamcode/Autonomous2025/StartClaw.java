package org.firstinspires.ftc.teamcode.Autonomous2025;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;

@Config
@Autonomous(name = "startClaw")
public class StartClaw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        Actions.runBlocking(claw.startClaw());
    }
}
