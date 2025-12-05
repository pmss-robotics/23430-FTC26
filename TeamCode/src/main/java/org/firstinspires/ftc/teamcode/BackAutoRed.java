//TODO: CHECK ALL VALS - NOTE: DOES NOT WORK!

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemNew;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;

@Config
@Autonomous(name="BackAutoRed", group="Auto")
public class BackAutoRed extends CommandOpMode {
    public double outtakeVal = 0.65;
    public static final String End_Pos = "X_End_Pos";

    @Override
    public void initialize() {
        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        BeltSubsystem belt = new BeltSubsystem(hardwareMap, telemetry);
        IntakeSubsystemNew intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap, telemetry);

        //TODO: UPDATE POINTS
        Pose2d start = new Pose2d(18, -73, Math.toRadians(90));
        Pose2d shootPos = new Pose2d(10, -65, Math.toRadians(66));

        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, start), telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        Action driveToShootPos = drive.actionBuilder(start)
                .splineToLinearHeading(shootPos, 0)
                .build();

        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ActionCommand(drive, driveToShootPos),
                                new InstantCommand(() -> outtake.setPower(outtakeVal), outtake)
                        ),
                        new WaitCommand(5500),

                        //turn on belt & intake to shoot
                        new InstantCommand(() -> kicker.moveToTarget(), kicker),
                        new InstantCommand(() -> belt.setPower(1.0), belt),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intake.setPower(1.0)),
                        new WaitCommand(2000),

                        new ParallelCommandGroup(
                                new InstantCommand(() -> kicker.moveToHome(), kicker),
                                new InstantCommand(() -> outtake.setPower(0.0), outtake),
                                new InstantCommand(() -> belt.setPower(0.0), belt),
                                new InstantCommand(() -> intake.setPower(0.0), intake)
                        )
                ) // <-- SequentialCommandGroup closes here
        );     // <-- schedule closes here
    }
}