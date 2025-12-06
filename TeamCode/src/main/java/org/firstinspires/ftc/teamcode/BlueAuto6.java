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
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemNew;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;

@Config
@Autonomous(name="BlueAuto6", group="Auto")
public class BlueAuto6 extends CommandOpMode {
    public static final String End_Pos = "X_End_Pos";

    @Override
    public void initialize() {
        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        BeltSubsystem belt = new BeltSubsystem(hardwareMap, telemetry);
        IntakeSubsystemNew intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap, telemetry);

        double outtakeVal = 0.54;

        Pose2d start = new Pose2d(35.38, -44, Math.toRadians(-40)); //start pos
        Pose2d shootPoint = new Pose2d(15, -10, Math.toRadians(-40)); //go to shoot pos
        Pose2d point3 = new Pose2d(19, 0, 0); //go to start of first row
        Pose2d point4 = new Pose2d(46, 0, 0); //go to end of first row
        Pose2d point5 = new Pose2d(15, -10, Math.toRadians(-40)); //go to shoot pos
        Pose2d point6 = new Pose2d(14, 23, 0); //go to start of second row
        Pose2d point7 = new Pose2d(50, 23, 0); //go to end of second row
        Pose2d point8 = new Pose2d(19, -3, Math.toRadians(-42)); //go to shoot pos

        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, start), telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        Action driveToShootPos = drive.actionBuilder(start)
                .splineToLinearHeading(shootPoint, 0)
                .build();

        Action driveTopoint3 = drive.actionBuilder(shootPoint)
                .splineToLinearHeading(point3, 0)
                .build();

        Action driveTopoint4 = drive.actionBuilder(point3)
                .splineToLinearHeading(point4, 0)
                .build();

        Action driveTopoint5 = drive.actionBuilder(point4)
                .splineToLinearHeading(point5, 0)
                .build();

        Action driveTopoint6 = drive.actionBuilder(point5)
                .splineToLinearHeading(point6, 0)
                .build();

        Action driveTopoint7 = drive.actionBuilder(point6)
                .splineToLinearHeading(point7, 0)
                .build();

        Action driveTopoint8 = drive.actionBuilder(point7)
                .splineToLinearHeading(point8, 0)
                .build();

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> kicker.moveToTarget(), kicker), //move kicker into position
                        //drive to the shooting position while turning on outtake
                        new ParallelCommandGroup(
                                new ActionCommand(drive, driveToShootPos),
                                new InstantCommand(() -> outtake.setPower(outtakeVal), outtake)
                        ),

                        //turn on belt & intake to shoot
                        new InstantCommand(() -> belt.setPower(1.0), belt),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intake.setPower(1.0)),
                        new WaitCommand(2000),

                        //turn off all subsystems
                        new ParallelCommandGroup(
                                new InstantCommand(() -> kicker.moveToHome(), kicker),
                                new InstantCommand(() -> outtake.setPower(0.0), outtake),
                                new InstantCommand(() -> belt.setPower(0.0), belt),
                                new InstantCommand(() -> intake.setPower(0.0), intake)
                        ),

                        new WaitCommand(2000),
                        //drive to point 3 and turn on subsystems to get ready for intaking
                        new ParallelCommandGroup(
                                new ActionCommand(drive, driveTopoint3),
                                new InstantCommand(() -> belt.setPower(0.6), belt),
                                new InstantCommand(() -> intake.setPower(0.5), intake)
                        ),

                        //drive to point 4 with subsystems on to intake
                        new ActionCommand(drive, driveTopoint4),

                        //turn belt & intake off, drive to shoot point and turn on outtake
                        new ParallelCommandGroup(
                                new InstantCommand(() -> kicker.moveToTarget(), kicker),
                                new InstantCommand(() -> belt.setPower(0.0), belt),
                                new InstantCommand(() -> intake.setPower(0.0), intake),
                                new ActionCommand(drive, driveTopoint5),
                                new InstantCommand(() -> outtake.setPower(outtakeVal), outtake)
                        ),

                        //turn on belt & intake to shoot, move the kicker back
                        new InstantCommand(() -> belt.setPower(0.7), belt),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intake.setPower(1.0)),
                        new WaitCommand(1000),
                        new InstantCommand(() -> kicker.moveToHome(), kicker),
                        new WaitCommand(2000),

                        //turn off subsystems
                        new ParallelCommandGroup(
                                new InstantCommand(() -> outtake.setPower(0.0), outtake),
                                new InstantCommand(() -> belt.setPower(0.0), belt),
                                new InstantCommand(() -> intake.setPower(0.0), intake)
                        ),

                        //drive to the second row to prepare for intaking
                        new ParallelCommandGroup(
                                new ActionCommand(drive, driveTopoint6),
                                new InstantCommand(() -> belt.setPower(0.5), belt),
                                new InstantCommand(() -> intake.setPower(0.7), intake)
                        ),

                        new WaitCommand(2000),
                        //drive forwards to intake balls
                        new ActionCommand(drive, driveTopoint7),

                        //move to shooting position, turn off belt & intake, turn on outtake
                        new ParallelCommandGroup(
                                new InstantCommand(() -> kicker.moveToTarget(), kicker),
                                new InstantCommand(() -> belt.setPower(0.0), belt),
                                new InstantCommand(() -> intake.setPower(0.0), intake),
                                new ActionCommand(drive, driveTopoint8),
                                new InstantCommand(() -> outtake.setPower(outtakeVal), outtake)
                        ),

                        //shoot
                        new InstantCommand(() -> belt.setPower(0.8), belt),
                        new WaitCommand(1000),
                        new InstantCommand(() -> intake.setPower(1.0)),
                        new WaitCommand(1000),
                        new InstantCommand(() -> kicker.moveToHome(), kicker),
                        new WaitCommand(2000),

                        //turn everything off
                        new ParallelCommandGroup(
                                new InstantCommand(() -> outtake.setPower(0.0), outtake),
                                new InstantCommand(() -> belt.setPower(0.0), belt),
                                new InstantCommand(() -> intake.setPower(0.0), intake)
                        ),

                        new InstantCommand(() -> blackboard.put(End_Pos, drive.getPose()))
                ) // <-- SequentialCommandGroup closes here
        );     // <-- schedule closes here
    }
}