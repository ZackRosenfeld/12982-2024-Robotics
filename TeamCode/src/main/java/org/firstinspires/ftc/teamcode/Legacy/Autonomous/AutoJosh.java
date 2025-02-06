package org.firstinspires.ftc.teamcode.Legacy.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Legacy.Autonomous.funWorld.PIDController;

public abstract class AutoJosh extends LinearOpMode {
    protected DcMotor leftfront = null;
    protected DcMotor leftback = null;
    protected DcMotor rightfront = null;
    protected DcMotor rightback = null;
    protected DcMotor lin1 = null;
    protected DcMotor lin2 = null;
    protected Servo claw = null;

    protected IMU imu = null;
    protected double zero_heading = 0.0;
    private Orientation angles = null;

    protected ElapsedTime runtime = new ElapsedTime();
    protected Telemetry telemetry_ = null;

    private double quitSpeed = 2.0; // Determines if encoder based wheels are taking too long
    private double speed = 0.77; // Used for @deprecated time based robot movement
    private double strafeSpd = 1.2; // Used for @deprecated time based robot movement
    private double turningCorrection = 0.001; // Used for @deprecated time based robot strafing
    private double turn90 = 0.73; // Used for @deprecated time based robot rotation
    protected double wheelPower = 0.8; // Wheel speed
    private double margin = 1; // Margin of error for gyrometer based robot rotation
    private final double ticksPerTile = 537.6/4/Math.PI*24*1.0; // Used for encoder based wheel movement (536.7 ticks/rev * 1/4pi rev/in * 24 in/tile * gear_ratio)

    private final double slidePower = 1; // speed
    private final double countsPerRevSlides = 2150.8; // Used to calculate counts per inch for the slide motors
    private final double inchesPerRevSlides = 4.72441; // Also used to calculate counts per inch for the slide motors
    private final double countsPerInchSlides = countsPerRevSlides / inchesPerRevSlides; // Converts motor ticks into inches of linear slide movement

    private final double clawOpen = .37; // position of the claw servo when fully open
    private final double clawClosed = .5; // position of the claw servo when fully closed
    private final double clawStarting = .3; // Used to make sure we fit at the start of the match
    /** Initialization **/
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        leftfront = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftback = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightfront = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightback = hardwareMap.get(DcMotor.class, "rightBackDrive");
        lin1 = hardwareMap.get(DcMotor.class, "linearSlide1");
        lin2 = hardwareMap.get(DcMotor.class, "linearSlide2");
        claw = hardwareMap.get(Servo.class, "clawClawServo");

        leftfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.FORWARD);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        rightback.setDirection(DcMotor.Direction.REVERSE);
        lin1.setDirection(DcMotor.Direction.REVERSE);
        lin2.setDirection(DcMotor.Direction.REVERSE);

        setWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setWheelMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry_ = telemetry;

        // Old IMU interface
        /*BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.mode = BNO055IMU.SensorMode.IMU;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(params);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;*/

        imu = hardwareMap.get(IMU.class, "imu");

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;

        // Set zero power behavior to brake/float.
        leftfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    /** Low-Level Conviniences **/
    public void leftWheels(double spd){
        leftfront.setPower(spd);
        leftback.setPower(spd);
    }
    public void rightWheels(double spd){
        rightfront.setPower(spd);
        rightback.setPower(spd);
    }


    /** Speed-Based Movement **/
    public void driveSpd(double spd){
        leftWheels(spd);
        rightWheels(spd);
    }
    public void turnRightSpd(double spd){
        leftWheels(spd);
        rightWheels(-spd);
    }
    public void turnLeftSpd(double spd){
        leftWheels(-spd);
        rightWheels(spd);
    }
    public void strafeLeftSpd(double spd, double corr){
        leftfront.setPower(-spd+corr);
        leftback.setPower(spd+corr);
        rightfront.setPower(spd-corr);
        rightback.setPower(-spd-corr);
    }
    public void strafeLeftSpd(double spd){
        strafeLeftSpd(spd, 0.0);
    }
    public void strafeRightSpd(double spd, double corr){
        leftfront.setPower(spd+corr);
        leftback.setPower(-spd+corr);
        rightfront.setPower(-spd-corr);
        rightback.setPower(spd-corr);
    }
    public void strafeRightSpd(double spd){
        strafeRightSpd(spd, 0.0);
    }


    /** Encoder-Based Movement **/
    public void move(double y_tiles, double x_tiles, String name){
        resetWheels();
        leftfront.setTargetPosition((int) ((y_tiles+x_tiles)*ticksPerTile));
        leftback.setTargetPosition((int) ((y_tiles-x_tiles)*ticksPerTile));
        rightfront.setTargetPosition((int) ((y_tiles-x_tiles)*ticksPerTile));
        rightback.setTargetPosition((int) ((y_tiles+x_tiles)*ticksPerTile));
        setWheelMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfront.setPower(wheelPower);
        leftback.setPower(wheelPower);
        rightfront.setPower(wheelPower);
        rightback.setPower(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (leftfront.isBusy() || rightfront.isBusy() || leftback.isBusy() || rightback.isBusy()) && runtime.seconds() < Math.sqrt(y_tiles*y_tiles+x_tiles*x_tiles)*quitSpeed/wheelPower){
            telemetry_.addData("Path", "%s: %s lf, %s rf, %s lb, %s rb", name, leftfront.getCurrentPosition(), rightfront.getCurrentPosition(), leftback.getCurrentPosition(), rightback.getCurrentPosition());
            telemetry_.update();
        }

        leftfront.setPower(0.0);
        leftback.setPower(0.0);
        rightfront.setPower(0.0);
        rightback.setPower(0.0);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double y_tiles, double x_tiles){
        move(y_tiles, x_tiles, "Moving");
    }

    public void strafeRight_enc(double tiles){
        move(0.0, -tiles, "Strafing Right");
    }
    public void strafeLeft_enc(double tiles){
        move(0.0, tiles, "Strafing Left");
    }
    public void forward_enc(double tiles){
        move(tiles, 0.0, "Forward");
    }
    public void backward_enc(double tiles){
        move(-tiles, 0.0, "Backward");
    }


    /** Encoder Utilities **/
    public void setWheelMode(DcMotor.RunMode mode){
        leftfront.setMode(mode);
        leftback.setMode(mode);
        rightfront.setMode(mode);
        rightback.setMode(mode);
    }
    public void resetWheels(){
        setWheelMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public double[] getVector(){
        double[] offset = new double[2];
        double lf = leftfront.getCurrentPosition() / ticksPerTile;
        double lb = leftback.getCurrentPosition() / ticksPerTile;
        double rf = rightfront.getCurrentPosition() / ticksPerTile;
        double rb = rightback.getCurrentPosition() / ticksPerTile;
        offset[0] = (lf + lb + rf + rb) / 4;
        offset[1] = (lf - lb - rf + rb) / 4;
        return offset;
    }

    /** PID Based Movements **/

    public void PIDMove(double x_tiles, double y_tiles, double speedDampener) {
        double margin = 5; // minimum difference between desired encoder position and actual encoder
        // position before robot is considered stable
        double stabilityTime = .5; // Time required in a stable state before method will quit
        boolean isStable = false; // Stores whether or not robot is stable to be printed

        final double Kp = 0.0019;
        final double Ki = 0;
        final double Kd = 0.0003;

        PIDController leftFrontController = new PIDController(Kp, Ki, Kd);
        PIDController leftBackController = new PIDController(Kp, Ki, Kd);
        PIDController rightFrontController = new PIDController(Kp, Ki, Kd);
        PIDController rightBackController = new PIDController(Kp, Ki, Kd);

        ElapsedTime stabilityTimer = new ElapsedTime(); // Timer used to determine if robot has been stable for the desired
        // amount of time. runtime will be used to track how long move has taken

        setWheelMode(DcMotor.RunMode.RUN_USING_ENCODER); // Making sure Wheels are in the correct runMode

        double leftFrontTarget = (y_tiles + x_tiles) * ticksPerTile; // Calculating target positions for each motor in ticks
        double leftBackTarget = (y_tiles - x_tiles) * ticksPerTile;  // This assumes mecanum wheels arranged in an X configuration
        double rightFrontTarget = (y_tiles - x_tiles) * ticksPerTile;
        double rightBackTarget = (y_tiles + x_tiles) * ticksPerTile;

        leftFrontController.setLastErrorInitial(leftFrontTarget - leftfront.getCurrentPosition());
        leftBackController.setLastErrorInitial(leftBackTarget - leftback.getCurrentPosition());
        rightFrontController.setLastErrorInitial(rightFrontTarget - rightfront.getCurrentPosition());
        rightBackController.setLastErrorInitial(rightBackTarget - rightback.getCurrentPosition());

        leftFrontController.resetTimerInitial();
        leftBackController.resetTimerInitial();
        rightFrontController.resetTimerInitial();
        rightBackController.resetTimerInitial();

        runtime.reset();
        stabilityTimer.reset();

        while (opModeIsActive() && stabilityTimer.seconds() <= stabilityTime) {

            double max;

            // Using PID controller to calculate motor powers
            double leftFrontPower = leftFrontController.PIDControl(leftFrontTarget, leftfront.getCurrentPosition());
            double leftBackPower = leftBackController.PIDControl(leftBackTarget, leftback.getCurrentPosition());
            double rightFrontPower = rightFrontController.PIDControl(rightFrontTarget, rightfront.getCurrentPosition());
            double rightBackPower = rightBackController.PIDControl(rightBackTarget, rightback.getCurrentPosition());

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Sending desired power to the motors
            leftfront.setPower(leftFrontPower * speedDampener);
            leftback.setPower(leftBackPower * speedDampener);
            rightfront.setPower(rightFrontPower * speedDampener);
            rightback.setPower(rightBackPower * speedDampener);

            isStable = (Math.abs(leftFrontTarget - leftfront.getCurrentPosition()) < margin
                    && Math.abs(leftBackTarget - leftback.getCurrentPosition()) < margin
                    && Math.abs(rightFrontTarget - rightfront.getCurrentPosition()) < margin
                    && Math.abs(rightBackTarget - rightback.getCurrentPosition()) < margin);

            if (!isStable) {
                stabilityTimer.reset();
            }

            telemetry_.addData("Motor Positions: ", "lf: %s, lb: %s, rf: %s, rb: %s",
                    leftfront.getCurrentPosition(), leftback.getCurrentPosition(),
                    rightfront.getCurrentPosition(), rightback.getCurrentPosition());
            telemetry_.addData("Motos Powers: ", "lf: %s, lb: %s, rf: %s, rb: %s",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry_.addData("stable: ", isStable);
            telemetry_.addData("Move time: ", runtime.seconds());
            telemetry_.update();
        }

        // Stop all wheel motors once the move is complete
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
    }




    /** Gyrometer Access **/
    public double getAngle(){
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360 + margin) % 360.0;
    }
    public double getNegAngle(){
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading - 720 - margin) % 360.0;
    }
    public double getSmAngle(){
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle - zero_heading + 360) % 360.0 - 180.0;
    }
    @Deprecated
    public void zeroHeading(){
        //double off = getAngle();
        //zero_heading += off;
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        zero_heading = angles.firstAngle;
    }


    /** Gyroscope-Based Rotation **/
    public void left_gyro(double quarters){
        if(quarters > 2){
            left_gyro(quarters-2);
            quarters -= 2;
        }
        double a = 0.0;
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        pause(0.05);
        while(opModeIsActive() && ((a=getAngle()) < 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        leftWheels(wheelPower*0.2);
        rightWheels(-wheelPower*0.2);
        while(opModeIsActive() && ((a=getAngle()) > 90*quarters+margin)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading + 90.0) % 360.0;
    }
    public void right_gyro(double quarters){
        if(quarters > 2){
            right_gyro(quarters-2);
            quarters -= 2;
        }
        double a = 0.0;
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && ((a=getNegAngle()%360) < -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        pause(0.05);
        while(opModeIsActive() && ((a=getNegAngle()) > -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        leftWheels(-wheelPower*0.2);
        rightWheels(wheelPower*0.2);
        while(opModeIsActive() && ((a=getNegAngle()) < -90*quarters-margin)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), a);
            telemetry_.update();
        }
        stopWheels();
        runtime.reset();
        zero_heading = (zero_heading - 90.0) % 360.0;
    }
    public void turnZero(double margin, String name){
        double a = getSmAngle();
        runtime.reset();
        while(Math.abs(a) > margin){
            leftWheels(a*0.005);
            rightWheels(-a*0.005);
            telemetry_.addData("Path", "%s: %2.5f S Elapsed, %2.3", name, a, runtime.seconds());
            telemetry_.update();
            a = getSmAngle();
        }
        stopWheels();
    }
    public void turnZero(double margin){
        turnZero(margin, "Turning");
    }
    public void turnZero(){
        turnZero(5.0);
    }


    /** Time-Based Movement (Backwards-Compatibility) **/
    //@Deprecated
    /*public void strafeLeft_time(double tiles){
        zeroHeading();
        strafeLeftSpd(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * strafeSpd / wheelPower)){
            telemetry_.addData("Path", "Strafing Left: %2.5f S Elapsed", runtime.seconds());
            strafeLeftSpd(wheelPower, getSmAngle()*turningCorrection);
        }
        stopWheels();
    }
    @Deprecated
    public void strafeRight_time(double tiles){
        zeroHeading();
        strafeRightSpd(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * strafeSpd / wheelPower)){
            telemetry_.addData("Path", "Strafing Right: %2.5f S Elapsed", runtime.seconds());
            strafeRightSpd(wheelPower, getSmAngle()*turningCorrection);
        }
        stopWheels();
    }*/
    @Deprecated
    public void forward_time(double tiles){
        leftWheels(wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void backward_time(double tiles){
        leftWheels(-wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < tiles * speed / wheelPower)){
            telemetry_.addData("Path", "Backward: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    /*@Deprecated
    public void left_time(double quarters){
        leftWheels(-wheelPower);
        rightWheels(wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Left: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        stopWheels();
    }
    @Deprecated
    public void right_time(double quarters){
        leftWheels(wheelPower);
        rightWheels(-wheelPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < quarters * turn90 / wheelPower)){
            telemetry_.addData("Path", "Turning Right: %2.5f S Elapsed, %2.3f deg", runtime.seconds(), getAngle());
            telemetry_.update();
        }
        stopWheels();
    }*/


    /** Movement Interface **/
    public void strafeRight(double tiles){
        strafeRight_enc(tiles*0.55);
    }
    public void strafeRight(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        strafeRight_enc(tiles*0.45);
        wheelPower = temp;
    }
    public void strafeLeft(double tiles){
        strafeLeft_enc(tiles*0.55);
    }
    public void strafeLeft(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        strafeLeft_enc(tiles*0.45);
        wheelPower = temp;
    }
    public void forward(double tiles){
        forward_enc(tiles*0.45);
    }
    public void forward(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        forward_enc(tiles*0.45);
        wheelPower = temp;
    }
    public void backward(double tiles){
        backward_enc(tiles*0.45);
    }
    public void backward(double tiles, double speed){
        double temp = wheelPower;
        wheelPower = speed;
        backward_enc(tiles*0.45);
        wheelPower = temp;
    }
    public void left(double quarters){
        left_gyro(quarters);
    }
    public void right(double quarters){
        right_gyro(quarters);
    }


    /** Mechanisms **/

    // Sets the run mode for both linear slide motors
    public void setSlideMode(DcMotor.RunMode mode) {
        lin1.setMode(mode);
        lin2.setMode(mode);
    }

    // moves the linear slides a specified distance in inches above their starting position
    public void moveSlides(double inches) {
        // calculating target position in motor ticks and sending it to both motors
        double targetPosition = (inches * countsPerInchSlides);
        lin1.setTargetPosition((int) targetPosition);
        lin2.setTargetPosition((int) targetPosition);

        setSlideMode(DcMotor.RunMode.RUN_TO_POSITION);

        // applying power which was specified earlier in the code
        lin1.setPower(slidePower);
        lin2.setPower(slidePower);

        // waiting for the move to complete
        while (opModeIsActive() && (lin1.isBusy() || lin2.isBusy())) {
            telemetry_.addData("Slide Positions:", "lin1: %s, lin2: %s", lin1.getCurrentPosition(), lin2.getCurrentPosition());
            telemetry_.update();
        }

        // stopping motors
        lin1.setPower(0);
        lin2.setPower(0);

        lin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // moves slides to specified distance in inches above the starting configuration
    // motor power can also be specified
    public void moveSlides(double inches, double speed) {
        // calculating target position in motor ticks and sending it to both motors
        double targetPosition = (inches * countsPerInchSlides);
        lin1.setTargetPosition((int) targetPosition);
        lin2.setTargetPosition((int) targetPosition);

        setSlideMode(DcMotor.RunMode.RUN_TO_POSITION);

        // applying power
        lin1.setPower(speed);
        lin2.setPower(speed);

        // waiting for the move to complete
        while (opModeIsActive() && lin1.isBusy() || lin2.isBusy()) {
            telemetry_.addData("Slide Positions:", "lin1: %s, lin2: %s", lin1.getCurrentPosition(), lin2.getCurrentPosition());
            telemetry_.update();
        }

        // stopping motors
        lin1.setPower(0);
        lin2.setPower(0);

        lin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
    @Deprecated
    public void armUp(){
        arm.setPower(0.5);
    }
    @Deprecated
    public void armDown(){
        arm.setPower(-0.5);
    }

    @Deprecated
    public void armUp(double pct){
        arm.setPower(armStatic+armPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct)){
            telemetry_.addData("Path", "Raising arm: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        arm.setPower(armStatic);
    }
    @Deprecated
    public void armDown(double pct){
        arm.setPower(armStatic-armPower);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < pct)){
        }
        arm.setPower(armStatic);
    }

    @Deprecated
    public void armUpEnc(int pos){
        runtime.reset();
        while(opModeIsActive() && (arm.getCurrentPosition() < pos) && (runtime.seconds() < Math.abs(pos-arm.getCurrentPosition())/100)){
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setPower(armPower);
        }
        arm.setPower(armStatic);
    }
    @Deprecated
    public void armDownEnc(int pos){
        runtime.reset();
        while(opModeIsActive() && (arm.getCurrentPosition() > pos) && (runtime.seconds() < Math.abs(pos-arm.getCurrentPosition())/100)){
            arm.setDirection(DcMotor.Direction.FORWARD);
            arm.setPower(armPower);
        }
        arm.setPower(armStatic);
    }

    // Encoder movement for arm
    public void armEnc(String posString){
        int pos = 0;
        if(posString == "top"){
            pos = 970;
        }
        else if(posString == "mid"){
            pos = 650;
        }
        else if(posString == "bot"){
            pos = 290;
        }
        else if(posString == "max"){
            pos = 1100;
        }
        else if(posString == "min"){
            pos = 0;
        }
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
        while(opModeIsActive() && arm.isBusy()) {
            telemetry_.addData("Arm", arm.getCurrentPosition());
            telemetry_.update();
        }
        arm.setPower(armStatic);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    */

    // Position based claw movement
    public void openClaw(){
        claw.setPosition(clawOpen);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Opening Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    public void closeClaw(){
        claw.setPosition(clawClosed);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Closing Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }

    // Puts the claw in the starting position
    // Specified earlier in the code
    public void startClaw() {
        claw.setPosition(clawStarting);
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 0.5){
            telemetry_.addData("Path", "Positioning Claw: %2.3f S elapsed", runtime.seconds());
            telemetry_.update();
        }
    }
    /*

    // Power based turner movement
    public void turner(double pct, double time) {
        turner.setPower(pct);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < time)){
            telemetry_.addData("Path", "Turner Spinning: %2.5f S Elapsed", runtime.seconds());
            telemetry_.update();
        }
        turner.setPower(0);
    }
  */


    /** Miscelaneous Utilities **/
    public void pause(double seconds){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            telemetry_.addData("Waiting", "%2.5f / %2.5f", runtime.seconds(), seconds);
            telemetry_.update();
        }
    }
    public void pause(double seconds, boolean telem){
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){
            if(telem) {
                telemetry_.addData("Waiting", "%2.5f / %2.5f", runtime.seconds(), seconds);
                telemetry_.update();
            }
        }
    }

    public void stopWheels(){
        leftWheels(0.0);
        rightWheels(0.0);
    }
}