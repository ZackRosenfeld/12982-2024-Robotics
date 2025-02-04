package org.firstinspires.ftc.teamcode.TeleOp2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Zack'sFuntasticTeleOp")
public class ZacksFuntasticTeleOp extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lin1;
    private DcMotor lin2;
    // Lift Movement constants
    public static double LIFT_POWER = 0.65; // power sent to lift motors
    private static final double COUNTS_PER_REV_LIFT = 2150.8/4;
    private static final double INCHES_PER_REV_LIFT = 4.72441;
    private static final double COUNTS_PER_INCH_LIFT = COUNTS_PER_REV_LIFT / INCHES_PER_REV_LIFT;
    private static final double LIFT_MAX_POSITION = 30;
    private static final double LIFT_MIN_POSITION = 0;
    private DcMotor armMotor;
    // Arm Movement Constants
    private static final double COUNTS_PER_REV_ARM = 5281.1;
    private static final double COUNTS_PER_DEGREE_ARM = COUNTS_PER_REV_ARM / 360.0;
    public static double ARM_POWER = .7;
    private static final double ARM_MAX_POSITION = 240;
    private static final double ARM_MIN_POSITION = 0;
    private static double armTargetPosition = 0;
    private static final double ARM_SPEED = 5; // maximum change in target position over 1 loop
    private Servo wristServo;
    private static double wristTargetPosition = 0;
    private Servo clawServo;
    private enum DriveMode {
        SLOW_MODE(.4, "Slow Mode"),
        FAST_MODE(.75, "Fast Mode");

        private final double speedControl;
        private final String displayName;

        DriveMode(double sc, String name) {
            speedControl = sc;
            displayName = name;
        }
    }
    private DriveMode driveMode = DriveMode.FAST_MODE;

    private enum ClawMode {
        OPEN_LOOP("Open Loop"),
        CLOSED_LOOP("Closed Loop");

        private final String displayName;

        ClawMode(String name) {
            displayName = name;
        }
    }
    private ClawMode clawMode = ClawMode.CLOSED_LOOP;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        armMotor = hardwareMap.get(DcMotor.class, "clawClawMotor");
        clawServo = hardwareMap.get(Servo.class, "clawClawServo");
        wristServo = hardwareMap.get(Servo.class, "clawWristServo"); //?
        telemetry.addData("Hardware: ", "Initialized");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        lin1.setDirection(DcMotor.Direction.FORWARD);
        lin2.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            // mode switching
            if (gamepad1.dpad_down) {
                driveMode = DriveMode.SLOW_MODE;
            }
            else if (gamepad1.dpad_up) {
                driveMode = DriveMode.FAST_MODE;
            }
            if (gamepad2.dpad_down) {
                clawMode = ClawMode.OPEN_LOOP;
            }
            else if (gamepad2.dpad_up) {
                clawMode = ClawMode.CLOSED_LOOP;
            }

            telemetry.addData("Drive Mode: ", driveMode.displayName);
            telemetry.addData("Claw Mode: ", clawMode.displayName);

            /** Drivetrain **/

            // taking input
            double a = -gamepad1.left_stick_y; //reverse the y stick
            double l = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double normalize = Math.max(Math.abs(a) + Math.abs(l) + Math.abs(rx), 1);

            // calculating wheel powers
            double frontLeftPower = (a + l + rx)/normalize*driveMode.speedControl;
            double backLeftPower = (a - l + rx)/normalize*driveMode.speedControl;
            double frontRightPower = (a - l - rx)/normalize*driveMode.speedControl;
            double backRightPower = (a + l - rx)/normalize*driveMode.speedControl;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if ((gamepad1.left_trigger > 0 && gamepad1.right_trigger >0) || (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0)) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }


            switch (clawMode) {
                case CLOSED_LOOP:
                    break;
                case OPEN_LOOP:
                    lin1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lin2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    // Lift Movements
                    if (-gamepad2.left_stick_y > 0 &&
                            lin1.getCurrentPosition() < LIFT_MAX_POSITION * COUNTS_PER_INCH_LIFT) {
                        lin1.setPower(LIFT_POWER * -gamepad2.left_stick_y);
                        lin2.setPower(LIFT_POWER * -gamepad2.left_stick_y);
                    }
                    else if (-gamepad2.left_stick_y < 0 &&
                            lin1.getCurrentPosition() > LIFT_MIN_POSITION * COUNTS_PER_INCH_LIFT) {
                        lin1.setPower(LIFT_POWER * -gamepad2.left_stick_y);
                        lin2.setPower(LIFT_POWER * -gamepad2.left_stick_y);
                    }
                    else {
                        lin1.setPower(0);
                        lin2.setPower(0);
                    }

                    // arm stuff
                    if (-gamepad2.right_stick_y > 0 &&
                            armMotor.getCurrentPosition() < ARM_MAX_POSITION * COUNTS_PER_DEGREE_ARM) {
                        armMotor.setPower(ARM_POWER * -gamepad2.right_stick_y);
                    }
                    else if (-gamepad2.right_stick_y < 0 &&
                            armMotor.getCurrentPosition() > ARM_MIN_POSITION * COUNTS_PER_DEGREE_ARM) {
                        armMotor.setPower(ARM_POWER * -gamepad2.right_stick_y);
                    }
                    else {
                        armMotor.setPower(0);
                    }


                    // claw stuff
                    if (gamepad2.right_trigger > 0) {
                        clawServo.setPosition(0.7);
                    }
                    else if (gamepad2.left_trigger > 0) {
                        clawServo.setPosition(0);
                    }

                    // Wrist Stuff
                    if (gamepad2.right_bumper) {
                        wristTargetPosition += .01;
                    }
                    else if (gamepad2.left_bumper) {
                        wristTargetPosition -= .01;
                    }

                    wristServo.setPosition(wristTargetPosition);
                    break;
            }
        }

    }
}
