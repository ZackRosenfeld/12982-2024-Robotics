package org.firstinspires.ftc.teamcode.AutoOp2025;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoOp2025.funWorld.PIDController;

@Autonomous(name="Autonomous2025")
public class Autonomous2025 extends LinearOpMode {

    // Hardware Variables
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lin1;
    private DcMotor lin2;
    private DcMotor clawMotor;
    private Servo clawServo;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Translation of Motor Ticks Into Real Inches
    static final double MAX_WHEEL_POWER = 1;  // Will not allow motor power to go above this value
                                              // Is not globally applied to power

    PIDController drive = new PIDController(.05 , 0, 0);

    @Override
    public void runOpMode() {

        // Intializing hardward
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        //lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        //lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        //clawMotor = hardwareMap.get(DcMotor.class, "clawClawMotor");
        //clawServo = hardwareMap.get(Servo.class, "clawClawServo");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        telemetry.addData("Hardware: ", "Initialized");

        // Setting motor directions, tweak as needed
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //lin1.setDirection(DcMotor.Direction.FORWARD);
        //lin2.setDirection(DcMotor.Direction.REVERSE);
        //clawMotor.setDirection(DcMotor.Direction.FORWARD);

        // changing motor settings
        // Using run without encoder will not actualy disable the ability to
        // Use encoders, but will disable the ability to get the velocity of a motor
        // This isn't necessary because a rough approximation will work and this
        // mode lets the motors run faster
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Im not sure which zero power behavior I will want so I will leave both for easy testing
        /*
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        DriveToPositionByPID(36, drive);

    }

    // Uses a PID controller to drive in a straight line to a position in inches
    // Only uses motor data from a single motor so may require tweaking
    // Uses variables stabilityTime and stabilityThreshold to determine when
    // a stable state has been reached
    public void DriveToPositionByPID(double inches, PIDController driver) {
        // Add a timer to track how long the error has stayed within the threshold
        ElapsedTime stabilityTimer = new ElapsedTime();
        double stabilityThreshold = 0.1; // The acceptable range of error (inches)
        double stabilityTime = 0.5; // Time (in seconds) the error must remain within threshold to be considered stable

        while (opModeIsActive()) {
            double currentPosition = frontLeft.getCurrentPosition() / COUNTS_PER_INCH;
            double error = Math.abs(inches - currentPosition);

            // PID control output
            double power = driver.PIDControl(inches, currentPosition);

            // Limit max power
            if (power > MAX_WHEEL_POWER) {
                power = MAX_WHEEL_POWER;
            }

            // Set motor power
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

            // Optional for testing and diagnosing issues
            telemetry.addData("Wheel Power: ", power);
            telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position: ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position: ", backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position: ", backRight.getCurrentPosition());
            telemetry.update();

            // Check if the error is within the threshold
            if (error < stabilityThreshold) {
                if (stabilityTimer.seconds() >= stabilityTime) {
                    // If error has been stable for enough time, exit the loop
                    break;
                }
            } else {
                // Reset the timer if the error goes above the threshold
                stabilityTimer.reset();
            }
        }

        // Stop all motors after reaching the target position and stabilizing
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


}
