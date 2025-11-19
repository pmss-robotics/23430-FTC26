package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KickerSubsystem extends SubsystemBase {

    //set up telemetry and servos
    Telemetry telemetry;
    ServoImplEx servoL;
    ServoImplEx servoR;

    //TODO: test HOME_POSITION & TARGET_POSITION values
    private final double HOME_POSITION_RIGHT = 0.0;
    private final double TARGET_POSITION_RIGHT = 0.07;
    private final double HOME_POSITION_LEFT = 1.0;
    private final double TARGET_POSITION_LEFT = 0.93;

    public KickerSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        servoL = hardwareMap.get(ServoImplEx.class, "servoL");
        servoR = hardwareMap.get(ServoImplEx.class, "servoR");

        servoL.setPosition(HOME_POSITION_LEFT);
        servoR.setPosition(HOME_POSITION_RIGHT);
    }

    public void moveToTarget() {
        servoR.setPosition(TARGET_POSITION_RIGHT);
        servoL.setPosition(TARGET_POSITION_LEFT);
    }

    public void moveToHome(){
        servoR.setPosition(HOME_POSITION_RIGHT);
        servoL.setPosition(HOME_POSITION_LEFT);
    }

    public double getPositionLeft() {return servoL.getPosition();}
    public double getPositionRight() {return servoR.getPosition();}

}
