package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

public class FlywheelSubsystem extends SubsystemBase {

    Telemetry telemetry;
    DcMotorEx outtakeMotor;

    private States.Outtake currentOuttakeState;
    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        outtakeMotor.setPower(clippedPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Outtake State", currentOuttakeState);
        telemetry.addData("Outtake Motor Power", outtakeMotor.getPower());
    }
}
