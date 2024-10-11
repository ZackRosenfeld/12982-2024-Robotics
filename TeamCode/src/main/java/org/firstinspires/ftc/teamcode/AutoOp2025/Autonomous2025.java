package org.firstinspires.ftc.teamcode.AutoOp2025;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;

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

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415); // Translation of Motor Ticks Into Real Inches
    static final double MAX_WHEEL_POWER = 1;  // Will not allow motor power to go above this value

    PIDController drive = new PIDController(.05 , 0, 0);

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        //lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        //lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        //clawMotor = hardwareMap.get(DcMotor.class, "clawClawMotor");
        //clawServo = hardwareMap.get(Servo.class, "clawClawServo");
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
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        DriveToPositionByPID(12, drive);

    }

    public void DriveToPositionByPID(double inches, PIDController driver) {
        while (Math.abs(inches - frontLeft.getCurrentPosition() / COUNTS_PER_INCH) > .1) {
            double power = driver.PIDControl(inches * COUNTS_PER_INCH, frontLeft.getCurrentPosition());

            if (power > MAX_WHEEL_POWER) {
                power = MAX_WHEEL_POWER;
            }

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);
        }

    }

}
