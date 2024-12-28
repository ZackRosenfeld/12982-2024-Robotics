package org.firstinspires.ftc.teamcode.Autonomous2025.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    // hardware
    Servo claw = null;

    // Closed and open positions of claw
    // Will need to be adjusted according to robot
    public final double CLAW_STARTING = .3; // used to fit inside size at the start of the match
    public final double CLAW_CLOSED = .35;
    public final double CLAW_OPEN = .37;

    // constructor
    // Configures servo
    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "clawClawServo");
    }
}
