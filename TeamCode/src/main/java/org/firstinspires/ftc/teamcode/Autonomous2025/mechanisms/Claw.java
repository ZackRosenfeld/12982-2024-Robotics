package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw {

    // hardware
    Servo claw = null;

    // Closed and open positions of claw
    // Will need to be adjusted according to robot
    public static double TEST_POSITION = 0; // used for dynamic testing with ftc dashboard
    public static double CLAW_STARTING = .3; // used to fit inside size at the start of the match
    public static double CLAW_CLOSED = .5;
    public static double CLAW_OPEN = .37;



    // constructor
    // Configures servo
    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "clawClawServo");
    }

    /** Utilities **/

    // returns current position of servo
    public double getPosition() {
        return claw.getPosition();
    }

    public class TestPosition implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(TEST_POSITION); // moving claw to closed position specified earlier
            telemetryPacket.put("Claw state:", "Closing");
            return false; // returning false causes action to end
        }
    }

    public Action testPosition() {
        return new TestPosition();
    }


    // Actions
    public class CloseClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(CLAW_CLOSED); // moving claw to closed position specified earlier
            telemetryPacket.put("Claw state:", "Closing");
            return false; // returning false causes action to end
        }
    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public class OpenClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(CLAW_OPEN); // moving claw to open position specified earlier
            telemetryPacket.put("Claw state:", "Opening");

            return false; // returning false causes action to end
        }
    }

    public Action openClaw() {
        return new OpenClaw();
    }

    public class StartClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(CLAW_STARTING); // moving claw to starting position specified earlier
            telemetryPacket.put("Claw state:", "Starting");

            return false; // returning false causes action to end
        }
    }

    public Action startClaw() {
        return new StartClaw();
    }

}
