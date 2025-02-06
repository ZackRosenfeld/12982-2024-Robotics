package org.firstinspires.ftc.teamcode.Autonomous2025;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Lift;
import org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms.Wrist;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class RightAuto2025 extends LinearOpMode {

    public static class PoseInitial {
        public double POSITION_X = 0;
        public double POSITION_Y = 0;
        public double HEADING = Math.PI / 2;
    }

    public static PoseInitial poseInitial = new PoseInitial();

    public static class Pose1 {
        public double POSITION_X = 0;
        public double POSITION_Y = 30;
        public double HEADING = Math.PI / 2;
    }

    public static Pose1 pose1 = new Pose1();



    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                new Pose2d(0, 0, Math.PI / 2));

    }
}
