package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {
    Servo wrist = null;

    public static double TEST_POSITION = 1;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "clawWristServo");
    }

    /** Utilites **/

    // returns current position of servo
    public double getPosition() {
        return wrist.getPosition();
    }

    public class SetPosition implements Action {

        public double POSITION = 0;

        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(POSITION);
            return false;
        }

    }

    public class TestPosition extends SetPosition {
        public TestPosition() {
            POSITION = TEST_POSITION;
        }
    }

    public Action testPosition() {
        return new TestPosition();
    }

    public class WristDown extends SetPosition {

        public WristDown() {
            POSITION = 1;
        }

    }


    public Action wristDown() {
        return new WristDown();
    }

    public class WristUp extends SetPosition {

        public WristUp() {
            POSITION = .3;
        }

    }


    public Action wristUp() {
        return new WristUp();
    }

    public class WallPickup extends SetPosition {

        public WallPickup() {
            POSITION = .65;
        }

    }


    public Action wallPickup() {
        return new WallPickup();
    }
}
