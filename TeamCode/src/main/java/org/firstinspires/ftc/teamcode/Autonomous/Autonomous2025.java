package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoTest")
public class Autonomous2025 extends AutoJosh {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize(hardwareMap, telemetry);

        waitForStart();

        PIDMove(1, 0, 0.6);

        PIDMove(0, 0, 0.6);

    }
}
