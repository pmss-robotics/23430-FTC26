package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class PusherSubsystem extends SubsystemBase {
    Telemetry telemetry;
    private final Servo servo;
    private final double HOME_POSITION = 1.0;
    private final double TARGET_POSITION = 0.5;

    public PusherSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, "pusher");
        servo.setPosition(HOME_POSITION);
    }

    public void moveToTarget() {
        servo.setPosition(TARGET_POSITION);
    }

    public void moveToHome() {
        servo.setPosition(HOME_POSITION);
    }

    public double getPosition() {
        return servo.getPosition();
    }
}
