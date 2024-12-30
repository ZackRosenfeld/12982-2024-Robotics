package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {
    Servo wrist = null;

    // movement constants
    public static double WRIST_DOWN = 0;
    public static double WRIST_UP = 1;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }
}
