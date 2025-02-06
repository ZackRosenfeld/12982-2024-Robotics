package org.firstinspires.ftc.teamcode.TeleOp2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Zack'sFuntasticTeleOp")
public class ZacksFuntasticTeleOp extends LinearOpMode {

    // Hardware Variables
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lin1;
    private DcMotor lin2;
    // Lift Movement constants
    public static double LIFT_POWER = 1; // power sent to lift motors
    private static final double COUNTS_PER_REV_LIFT = 2150.8/4;
    private static final double INCHES_PER_REV_LIFT = 4.72441;
    private static final double COUNTS_PER_INCH_LIFT = COUNTS_PER_REV_LIFT / INCHES_PER_REV_LIFT;
    private static final double LIFT_MAX_POSITION = 30;
    private static final double LIFT_MIN_POSITION = 0;
    private double liftTargetPosition = 0;
    private static final double LIFT_SPEED = 10; // maximum change in target position per second in inches
    private DcMotor armMotor;
    // Arm Movement Constants
    private static final double COUNTS_PER_REV_ARM = 5281.1;
    private static final double COUNTS_PER_DEGREE_ARM = COUNTS_PER_REV_ARM / 360.0;
    public static double ARM_POWER = .7;
    private static final double ARM_MAX_POSITION = 240;
    private static final double ARM_MIN_POSITION = 0;
    private static double armTargetPosition = 0;
    private static final double ARM_SPEED = 180; // maximum change in target position over 1 second in degrees
    private Servo wristServo;
    private static double wristTargetPosition = 0;
    private Servo clawServo;
    // Drive mode enum which also stores speedControl and Name to be displayed
    // speedControl is multiplied by each motor power
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

    /*
     * When we added more complex mechanisms to our robot
     * we wanted to make it easier for the drivers by making
     * single buttons make multiple mechanisms move to key positions.
     *
     * We also didn't want to loose the flexibility of having full, fine control over
     * each mechanism. So, we added Claw modes, where one will have the new control
     * scheme with automated movements, while the other will still allow fine control
     * over all mechanisms.
     */
    private enum ClawMode {
        OPEN_LOOP("Open Loop"),
        CLOSED_LOOP_SPECIMENS("Closed Loop (Specimens)"),
        CLOSED_LOOP_SAMPLES("Closed Loop (Samples)");

        private final String displayName;

        ClawMode(String name) {
            displayName = name;
        }
    }
    private ClawMode clawMode = ClawMode.CLOSED_LOOP_SPECIMENS;

    // Class which denotes a position of Lift, Arm, and Wrist
    // Call updateTargetPosition to set respective target positions to stored values
    // Each pose used in the opMode should be an object of this class
    private class Pose {
        private double LIFT_POSITION;
        private double ARM_POSITION;
        private double WRIST_POSITION;

        public Pose(double liftPos, double armPos, double wristPos) {
            LIFT_POSITION = liftPos;
            ARM_POSITION = armPos;
            WRIST_POSITION = wristPos;
        }

        public void updateTargetPositions() {
            liftTargetPosition = LIFT_POSITION;
            armTargetPosition = ARM_POSITION;
            wristTargetPosition = WRIST_POSITION;
        }
    }

    // Stored Poses
    // These store every position that Closed Loop mode runs to
    // and should be tweaked until they work
    Pose home = new Pose(0, 0, 0);
    Pose belowHighBar = new Pose(0, 170, 1);
    Pose scoringHighBar = new Pose(0, 170, .3);
    Pose specimenFromWall = new Pose(0, 30, .65);

    // ElapsedTime to keep speeds consistent regardless of loop time
    ElapsedTime loopTime = new ElapsedTime();
    private double lastLoopTime = 0; // The time that the loop takes is stored at the end
                                     // To prevent the loopTime used for movements increasing
                                     // Over the course of a loop


    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware configuration
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        armMotor = hardwareMap.get(DcMotor.class, "clawClawMotor"); // Why
        clawServo = hardwareMap.get(Servo.class, "clawClawServo");
        wristServo = hardwareMap.get(Servo.class, "clawWristServo"); //?
        telemetry.addData("Hardware: ", "Initialized");

        // Setting behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Setting Directions
        backRight.setDirection(DcMotor.Direction.REVERSE);
        lin1.setDirection(DcMotor.Direction.FORWARD);
        lin2.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            loopTime.reset();

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
            else if (gamepad2.dpad_right) {
                clawMode = ClawMode.CLOSED_LOOP_SPECIMENS;
            }
            else if (gamepad2.dpad_left) {
                clawMode = ClawMode.CLOSED_LOOP_SAMPLES;
            }

            telemetry.addData("Drive Mode: ", driveMode.displayName);
            telemetry.addData("Claw Mode: ", clawMode.displayName);

            /** Drivetrain **/

            // taking input
            double a = -gamepad1.left_stick_y; //reverse the y stick
            double l = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double normalize = Math.max(Math.abs(a) + Math.abs(l) + Math.abs(rx), 1); // Make sure no wheel power exceeds 1

            // calculating wheel powers
            double frontLeftPower = (a + l + rx)/normalize*driveMode.speedControl;
            double backLeftPower = (a - l + rx)/normalize*driveMode.speedControl;
            double frontRightPower = (a - l - rx)/normalize*driveMode.speedControl;
            double backRightPower = (a + l - rx)/normalize*driveMode.speedControl;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // E Brake
            if ((gamepad1.left_trigger > 0 && gamepad1.right_trigger >0) || (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0)) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }


            switch (clawMode) {
                case CLOSED_LOOP_SPECIMENS:

                    // Taking Input
                    if (gamepad2.back) {
                        home.updateTargetPositions();
                    }
                    else if (gamepad2.a) {
                        belowHighBar.updateTargetPositions();
                    }
                    else if (gamepad2.b) {
                        scoringHighBar.updateTargetPositions();
                    }
                    else if (gamepad2.x) {
                        specimenFromWall.updateTargetPositions();
                    }

                    // movements
                    lin1.setTargetPosition((int) liftTargetPosition);
                    lin2.setTargetPosition((int) liftTargetPosition);

                    lin1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    lin1.setPower(LIFT_POWER);
                    lin2.setPower(LIFT_POWER);


                    armMotor.setTargetPosition((int) armTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ARM_POWER);


                    wristServo.setPosition(wristTargetPosition);

                    // Claw remains fully controllable
                    if (gamepad2.right_trigger > 0) {
                        clawServo.setPosition(0.7); // Closed
                    }
                    else if (gamepad2.left_trigger > 0) {
                        clawServo.setPosition(0); // Open
                    }

                    break;
                case CLOSED_LOOP_SAMPLES:
                    // TODO: everything
                    break;
                case OPEN_LOOP:

                    // Lift Movements
                    // uses run to position to hopefully correct for lifts dropping when motors are not powered
                    // LIFT_MAX_POSITION and LIFT_MIN_POSITION will need to be tweaked
                    if (-gamepad2.left_stick_y > 0 &&
                            lin1.getCurrentPosition() < LIFT_MAX_POSITION * COUNTS_PER_INCH_LIFT) {
                        liftTargetPosition += LIFT_SPEED * lastLoopTime * COUNTS_PER_INCH_LIFT * -gamepad2.left_stick_y;
                        // Long equation which should take into account loop time and stick position
                    }
                    else if (-gamepad2.left_stick_y < 0 &&
                            lin1.getCurrentPosition() > LIFT_MIN_POSITION * COUNTS_PER_INCH_LIFT) {
                        liftTargetPosition += LIFT_SPEED * lastLoopTime * COUNTS_PER_INCH_LIFT * -gamepad2.left_stick_y;
                    }

                    // Lift Movements
                    lin1.setTargetPosition((int) liftTargetPosition);
                    lin2.setTargetPosition((int) liftTargetPosition);

                    lin1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lin2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    lin1.setPower(LIFT_POWER);
                    lin2.setPower(LIFT_POWER);


                    // arm stuff
                    if (-gamepad2.right_stick_y > 0 &&
                            armMotor.getCurrentPosition() < ARM_MAX_POSITION * COUNTS_PER_DEGREE_ARM) {
                        armTargetPosition += ARM_SPEED * lastLoopTime * COUNTS_PER_DEGREE_ARM * -gamepad2.right_stick_y;
                    }
                    else if (-gamepad2.right_stick_y < 0 &&
                            armMotor.getCurrentPosition() > ARM_MIN_POSITION * COUNTS_PER_DEGREE_ARM) {
                        armTargetPosition += ARM_SPEED * lastLoopTime * COUNTS_PER_DEGREE_ARM * -gamepad2.right_stick_y;
                    }

                    // arm movements
                    armMotor.setTargetPosition((int) armTargetPosition);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(ARM_POWER);


                    // claw stuff
                    if (gamepad2.right_trigger > 0) {
                        clawServo.setPosition(0.7);
                    }
                    else if (gamepad2.left_trigger > 0) {
                        clawServo.setPosition(0);
                    }

                    // Wrist Stuff
                    if (gamepad2.right_bumper) {
                        wristTargetPosition += .5 * lastLoopTime;
                    }
                    else if (gamepad2.left_bumper) {
                        wristTargetPosition -= .5 * lastLoopTime;
                    }

                    wristServo.setPosition(wristTargetPosition);
                    break;
            }

            // Handling timer and telemetry
            lastLoopTime = loopTime.seconds();

            loopTime.reset();

            telemetry.update();
        }

    }
}
