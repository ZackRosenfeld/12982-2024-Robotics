package org.firstinspires.ftc.teamcode.Autonomous2025.TestAutos.PositionTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "LiftPositionTest", group = "Test_Autos")
public class LiftPositionTest extends LinearOpMode {

    private DcMotorEx lin1 = null;
    private DcMotorEx lin2 = null;
    public static double POSITION = 20; // position lifts will run to in inches
                                        // configurable with ftc dashboard
    private final double COUNTS_PER_INCH_LIFT = 2150.8/4/4.72441;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware setup
        lin1 = hardwareMap.get(DcMotorEx.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotorEx.class, "linearSlide2");

        lin1.setDirection(DcMotorEx.Direction.FORWARD);
        lin2.setDirection(DcMotorEx.Direction.REVERSE);

        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        TelemetryPacket telemetryPacket = new TelemetryPacket();

        waitForStart();

        // running to position and logging
        while (opModeIsActive()) {
            lin1.setTargetPosition((int) (POSITION * COUNTS_PER_INCH_LIFT));
            lin2.setTargetPosition((int) (POSITION * COUNTS_PER_INCH_LIFT));

            lin1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            lin2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            lin1.setPower(.4);
            lin2.setPower(.4);

            telemetryPacket.put("liftPos: ", lin1.getCurrentPosition());

        }
    }
}
