package org.firstinspires.ftc.teamcode.TeleOp2025;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp2025")
public class TeleOp2025 extends OpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lin1;
    private DcMotor lin2;
    private DcMotor clawMotor;
    private Servo clawServo;
    private final double speed = 0.7;
    @Override
    public void init(){
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        clawMotor = hardwareMap.get(DcMotor.class, "clawClawMotor");
        clawServo = hardwareMap.get(Servo.class, "clawClawServo");
        telemetry.addData("Hardware: ", "Initialized");
    }
    @Override
    public void loop(){
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

        //linear slide time
        if (gamepad2.left_stick_y > 0)
        {
            lin1.setPower(speed);
            lin2.setPower(speed);
        }
        else if (gamepad2.left_stick_y < 0)
        {
            lin1.setPower(-speed);
            lin2.setPower(-speed);
        }

        //arm time
        if(gamepad2.right_stick_y > 0)
        {
            clawMotor.setPower(speed);
        }
        else if (gamepad2.right_stick_y > 0)
        {
            clawMotor.setPower(-speed);
        }
        //claw time
        if (gamepad2.left_trigger > 0)
        {
            clawServo.setPosition(0.4);
        }
        else if (gamepad2.right_trigger > 0)
        {
            clawServo.setPosition(0.7);
        }
        //SET BACK TO ZERO NO MORE MOVEMENT
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        lin1.setPower(0);
        lin2.setPower(0);
    }
}
