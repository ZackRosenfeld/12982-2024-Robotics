package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    Servo wrist = null;

    // movement constants
    public final double WRIST_DOWN = 0;
    public final double WRIST_UP = 1;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }
}
