package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//TODO: delete old intake subsystem after testing
public class IntakeSubsystemNew extends SubsystemBase {
    Telemetry telemetry;
    DcMotorEx intakeMotor;

    public IntakeSubsystemNew(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        double clippedPower = Range.clip(power, -1.0, 1.0);
        intakeMotor.setPower(clippedPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Motor Power: ", intakeMotor.getPower());
    }
}
