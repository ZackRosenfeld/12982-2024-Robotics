package org.firstinspires.ftc.teamcode.TeleOp2025;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp2025")
public class TeleOp2025 extends OpMode {
    //initialization of motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lin1;
    private DcMotor lin2;
    //private DcMotor clawMotor;
    private Servo clawServo;
    private double speedControl = 2; //
    @Override
    public void init(){
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        //clawMotor = hardwareMap.get(DcMotor.class, "clawClawMotor");
        clawServo = hardwareMap.get(Servo.class, "clawClawServo");
        telemetry.addData("Hardware: ", "Initialized");

        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    @Override
    public void loop(){
        backRight.setDirection(DcMotor.Direction.REVERSE);
        lin1.setDirection(DcMotor.Direction.REVERSE);
        lin2.setDirection(DcMotor.Direction.FORWARD);
        double a = -gamepad1.left_stick_y; //reverse the y stick
        double l = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double slideSpeed = .7;
        //double speedControl = Math.max(Math.abs(a) + Math.abs(l) + Math.abs(rx), 1);

        //slow mode
        if (gamepad1.dpad_down)
        {
            speedControl = 4;
        }
        else if (gamepad1.dpad_up)
        {
            speedControl = 2;
        }

        //mechanics
        double frontLeftPower = (a + l + rx)/speedControl;
        double backLeftPower = (a - l + rx)/speedControl;
        double frontRightPower = (a - l - rx)/speedControl;
        double backRightPower = (a + l - rx)/speedControl;



        //basic drive
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


        //EMERGENCY BRAKE
        if ((gamepad1.left_trigger > 0 && gamepad1.right_trigger >0) || (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0)) {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }

        //linear slide time
        if (gamepad2.left_stick_y > 0)
        {
            lin1.setPower(slideSpeed);
            lin2.setPower(slideSpeed);
        }
        else if (gamepad2.left_stick_y < 0)
        {
            lin1.setPower(-slideSpeed);
            lin2.setPower(-slideSpeed);
        }
        else {
            lin1.setPower(0);
            lin2.setPower(0);
        }

        //arm time
        /*if(gamepad2.right_stick_y > 0)
        {
            clawMotor.setPower(1);
        }
        else if (gamepad2.right_stick_y < 0)
        {
            clawMotor.setPower(-1);
        }*/
        //claw time
        if (gamepad2.left_trigger > 0)
        {
            clawServo.setPosition(0.37);
        }
        else if (gamepad2.right_trigger > 0)
        {
            clawServo.setPosition(0.5);
        }
        else if (gamepad2.dpad_up)
        {
            clawServo.setPosition(0.3);
        }
        //SET BACK TO ZERO NO MORE MOVEMENT
        //lin1.setPower(0);
        //lin2.setPower(0);
        //clawMotor.setPower(0);
    }
}