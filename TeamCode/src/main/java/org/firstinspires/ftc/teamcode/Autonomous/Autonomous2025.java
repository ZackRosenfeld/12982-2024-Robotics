package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Autonomous.funWorld.PIDController;
import java.util.List;

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
    private IMU imu;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Translation of Motor Ticks Into Real Inches
    static final double MAX_WHEEL_POWER = 1;  // Will not allow motor power to go above this value
                                              // Is not globally applied to power

    PIDController drive = new PIDController(.07 , 0, 0);

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
        imu = hardwareMap.get(IMU.class, "imu");

        // Utilizing bulk reads to speed up code processing
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Hardware: ", "Initialized");

        // Setting motor directions, tweak as needed
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        //lin1.setDirection(DcMotor.Direction.FORWARD);
        //lin2.setDirection(DcMotor.Direction.REVERSE);
        //clawMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // IMU orientation setup
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        DriveToPositionByPID(36, drive);
        StrafeToPositionByPID(36, drive);

    }


    // Uses a PID controller to drive in a straight line right to a position in inches
    // Only uses motor data from a single motor so may require tweaking
    // Uses variables stabilityTime and stabilityThreshold to determine when
    // a stable state has been reached
    public void StrafeToPositionByPID(double inches, PIDController driver) {
        // Add a timer to track how long the error has stayed within the threshold
        ElapsedTime stabilityTimer = new ElapsedTime();
        double stabilityThreshold = 0.1; // The acceptable range of error (inches)
        double stabilityTime = 0.5; // Time (in seconds) the error must remain within threshold to be considered stable

        while (opModeIsActive()) {
            double frontRightPosition = frontRight.getCurrentPosition() / COUNTS_PER_INCH;
            double frontLeftPosition = frontLeft.getCurrentPosition() / COUNTS_PER_INCH;
            double backRightPosition = backRight.getCurrentPosition() / COUNTS_PER_INCH;
            double backLeftPosition = backLeft.getCurrentPosition() / COUNTS_PER_INCH;

            // PID control output
            double frontLeftPower = driver.PIDControl(inches, frontLeftPosition);
            double backLeftPower = driver.PIDControl(-inches, backLeftPosition);
            double frontRightPower = driver.PIDControl(-inches, frontRightPosition);
            double backRightPower = driver.PIDControl(inches, backRightPosition);

            // Limit max power
            if (frontLeftPower > MAX_WHEEL_POWER) {
                frontLeftPower = MAX_WHEEL_POWER;
            }
            else if (frontLeftPower < -MAX_WHEEL_POWER) {
                frontLeftPower = -MAX_WHEEL_POWER;
            }
            if (backLeftPower > MAX_WHEEL_POWER) {
                backLeftPower = MAX_WHEEL_POWER;
            }
            else if (backLeftPower < -MAX_WHEEL_POWER) {
                backLeftPower = -MAX_WHEEL_POWER;
            }
            if (frontRightPower > MAX_WHEEL_POWER) {
                frontRightPower = MAX_WHEEL_POWER;
            }
            else if (frontRightPower < -MAX_WHEEL_POWER) {
                frontRightPower = -MAX_WHEEL_POWER;
            }
            if (backRightPower > MAX_WHEEL_POWER) {
                backRightPower = MAX_WHEEL_POWER;
            }
            else if (backRightPower < -MAX_WHEEL_POWER) {
                backRightPower = -MAX_WHEEL_POWER;
            }

            // Set motor power
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Optional for testing and diagnosing issues
            telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position: ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position: ", backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position: ", backRight.getCurrentPosition());
            telemetry.update();

            // Check if the error is within the threshold
            if (frontLeftPosition - inches < stabilityThreshold && backLeftPosition + inches < stabilityThreshold
                    && frontRightPosition + inches < stabilityThreshold
                    && backRightPosition - inches < stabilityThreshold) {
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void DriveToPositionByPID(double inches, PIDController driver) {
        // Add a timer to track how long the error has stayed within the threshold
        ElapsedTime stabilityTimer = new ElapsedTime();
        double stabilityThreshold = 0.1; // The acceptable range of error (inches)
        double stabilityTime = 0.5; // Time (in seconds) the error must remain within threshold to be considered stable

        while (opModeIsActive()) {
            double frontRightPosition = frontRight.getCurrentPosition() / COUNTS_PER_INCH;
            double frontLeftPosition = frontLeft.getCurrentPosition() / COUNTS_PER_INCH;
            double backRightPosition = backRight.getCurrentPosition() / COUNTS_PER_INCH;
            double backLeftPosition = backLeft.getCurrentPosition() / COUNTS_PER_INCH;

            // PID control output
            double frontLeftPower = driver.PIDControl(inches, frontLeftPosition);
            double backLeftPower = driver.PIDControl(inches, backLeftPosition);
            double frontRightPower = driver.PIDControl(inches, frontRightPosition);
            double backRightPower = driver.PIDControl(inches, backRightPosition);

            // Limit max power
            if (frontLeftPower > MAX_WHEEL_POWER) {
                frontLeftPower = MAX_WHEEL_POWER;
            }
            else if (frontLeftPower < -MAX_WHEEL_POWER) {
                frontLeftPower = -MAX_WHEEL_POWER;
            }
            if (backLeftPower > MAX_WHEEL_POWER) {
                backLeftPower = MAX_WHEEL_POWER;
            }
            else if (backLeftPower < -MAX_WHEEL_POWER) {
                backLeftPower = -MAX_WHEEL_POWER;
            }
            if (frontRightPower > MAX_WHEEL_POWER) {
                frontRightPower = MAX_WHEEL_POWER;
            }
            else if (frontRightPower < -MAX_WHEEL_POWER) {
                frontRightPower = -MAX_WHEEL_POWER;
            }
            if (backRightPower > MAX_WHEEL_POWER) {
                backRightPower = MAX_WHEEL_POWER;
            }
            else if (backRightPower < -MAX_WHEEL_POWER) {
                backRightPower = -MAX_WHEEL_POWER;
            }

            // Set motor power
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Optional for testing and diagnosing issues
            telemetry.addData("Front Left Position: ", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position: ", frontRight.getCurrentPosition());
            telemetry.addData("Back Left Position: ", backLeft.getCurrentPosition());
            telemetry.addData("Back Right Position: ", backRight.getCurrentPosition());
            telemetry.update();

            // Check if the error is within the threshold
            if (frontLeftPosition - inches < stabilityThreshold && backLeftPosition - inches < stabilityThreshold
                    && frontRightPosition - inches < stabilityThreshold
                    && backRightPosition - inches < stabilityThreshold) {
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void TurnByPID(double degrees, PIDController turner) {
        // Add a timer to track how long the error has stayed within the threshold
        ElapsedTime stabilityTimer = new ElapsedTime();
        double stabilityThreshold = 1; // The acceptable range of error (degrees)
        double stabilityTime = 0.5; // Time (in seconds) the error must remain within threshold to be considered stable

        while (opModeIsActive()) {

            // Get the angles from the IMU and store the Yaw as angle
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            double currentAngle = orientation.getYaw();

            double power = turner.PIDControl(degrees, currentAngle);

            if (power > MAX_WHEEL_POWER) {
                power = MAX_WHEEL_POWER;
            }
            else if (power < -MAX_WHEEL_POWER) {
                power = -MAX_WHEEL_POWER;
            }

            // Sending power to the motors. Make sure Left is being driven foward and Right is being driven backwards for a positive angle. otherwise flip the signs
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            if (Math.abs(degrees - currentAngle) < stabilityThreshold) {
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


}
