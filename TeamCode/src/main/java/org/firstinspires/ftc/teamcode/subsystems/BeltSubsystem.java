package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
//aaa
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

public class BeltSubsystem extends SubsystemBase {

    Telemetry telemetry;
    DcMotorEx beltMotor;

    private States.Outtake currentOuttakeState;
    public BeltSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        beltMotor = hardwareMap.get(DcMotorEx.class, "belt");

        currentOuttakeState = States.Outtake.home;
    }

    public States.Outtake getCurrentOuttakeState() {
        return currentOuttakeState;
    }

    public void setOuttakeState(States.Outtake state) {
        currentOuttakeState = state;
    }

    public void setPower(double power) {
        double clippedPower = Range.clip(power, -1.0, 1.0);
        beltMotor.setPower(clippedPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Belt subsystem", currentOuttakeState);
        telemetry.addData("Belt subsystem", beltMotor.getPower());
    }
}