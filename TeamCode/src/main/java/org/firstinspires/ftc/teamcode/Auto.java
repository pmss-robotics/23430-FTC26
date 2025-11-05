package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Config
@Autonomous(name="Auto", group="Auto")
public class Auto extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        Pose2d start = new Pose2d(0,0,0);

        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, start), telemetry);

        // set with your own target poses and headings.
        Pose2d end = new Pose2d(0,0,0);
        Action square = drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(24)
                .turn(Math.toRadians(90))
                // you can add more splines here instead of creating a path2,3,4,etc.
                // make a new distinct path when you need to
                .build();

        SequentialCommandGroup routine = new SequentialCommandGroup(
                new ActionCommand(drive, square)
        );
        schedule(routine);
    }
}