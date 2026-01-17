package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TransferSubsystem extends SubsystemBase {

    Telemetry telemetry;
    DcMotorEx transferMotor;

    public TransferSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        double clippedPower = Range.clip(power, -1.0, 1.0);
        transferMotor.setPower(clippedPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Transfer motor power: ", transferMotor.getPower());
    }
}