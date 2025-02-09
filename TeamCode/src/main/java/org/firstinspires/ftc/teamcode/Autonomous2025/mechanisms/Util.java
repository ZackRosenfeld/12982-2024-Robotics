package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Util {

    public class Pause implements Action {

        ElapsedTime timer = new ElapsedTime();
        public boolean initialized = false;
        public double waitSeconds;

        public Pause(double seconds) {
            waitSeconds = seconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();

                initialized = true;
            }

            if (timer.seconds() >= waitSeconds) {
                return true;
            }
            else {
                return false;
            }
        }
    }

    public Action pause(double seconds) {
        return new Pause(seconds);
    }

}
