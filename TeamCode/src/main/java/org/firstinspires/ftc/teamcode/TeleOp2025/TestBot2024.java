package org.firstinspires.ftc.teamcode.TeleOp2025;
//Plow Bot Code

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TestBot2024")
public class TestBot2024 extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor clawElbow;
    private DcMotor clawWrist;
    private Servo clawClaw;
    double ticks = 2786.2;
    //ticks subject to change!!
    double newTarget;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        backRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        clawElbow = hardwareMap.get(DcMotor.class, "motorClawElbow");
        clawWrist = hardwareMap.get(DcMotor.class, "motorClawWrist");
        clawClaw = hardwareMap.get(Servo.class, "clawClaw");
        telemetry.addData("Hardware: ", "Initialized");

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; //reverse the y stick
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double speedControl = 1.5;

        //mechanics
        double frontLeftPower = (y + x + rx)/speedControl;
        double backLeftPower = (y - x + rx)/speedControl;
        double frontRightPower = (-(y - x - rx))/speedControl;
        double backRightPower = (-(y + x - rx))/speedControl;

        //basic drive
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        //rotations
        frontLeft.setPower(y + rx);
        backLeft.setPower(y + rx);
        frontRight.setPower(-(y - rx));
        backRight.setPower(-(y - rx));

        //claw elbow
        if (gamepad2.left_stick_y > 0)
        {
            clawElbow.setPower(1);
        }
        if (gamepad2.left_stick_y < 0)
        {
            clawElbow.setPower(-1);
        }
        //claw wrist
        if (gamepad2.right_stick_y > 0)
        {
            clawWrist.setPower(0.5);
        }
        if (gamepad2.right_stick_y < 0)
        {
            clawWrist.setPower(-0.5);
        }
        //claw grabber!!!
        if (gamepad2.left_trigger > 0)
        {
            clawClaw.setPosition(0.5);
        }
        if (gamepad2.right_trigger > 0)
        {
            clawClaw.setPosition(-0.75);
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        clawElbow.setPower(0);
        clawWrist.setPower(0);

    }
}

