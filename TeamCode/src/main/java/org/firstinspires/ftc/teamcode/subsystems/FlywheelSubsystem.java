package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final DcMotorEx outtakeMotor;

    // PIDF coefficients
    private static final double kP = 29.0, kI = 0.0, kD = 0.2;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;
    //private static final double kF = 32767.0 / ((MAX_RPM * TICKS_PER_REV) / 60.0);
    private static final double kF = 14.0;

    private double targetRpm = 0.0;

    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public void setPower(double power) {
        outtakeMotor.setPower(Range.clip(power, -1.0, 1.0));
        targetRpm = 0.0;
    }

    public void setVelocityRpm(double rpm) {
        targetRpm = rpm;
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        outtakeMotor.setVelocity(ticksPerSec);
    }

    @Override
    public void periodic() {
        double actualRpm = (outtakeMotor.getVelocity() * 60.0) / TICKS_PER_REV;
        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Actual RPM", actualRpm);
    }
}
