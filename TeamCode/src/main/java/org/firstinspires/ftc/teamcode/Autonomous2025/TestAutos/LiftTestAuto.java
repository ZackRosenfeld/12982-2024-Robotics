package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;

@Autonomous(name = "LiftTest", group = "Test_Autos")
public class LiftTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);

        waitForStart();

        Actions.runBlocking(new SequentialAction(lift.aboveHighBar(),
                lift.scoringHighBar(),
                lift.aboveHighBar(),
                lift.down()));
    }
}
