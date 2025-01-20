package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Arm;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmPositionTest extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        Actions.runBlocking(arm.testPosition());
    }
}
