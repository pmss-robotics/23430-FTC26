package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PusherSubsystem;

@Config
@Autonomous(name="Auto", group="Auto")
public class Auto extends CommandOpMode {
    Pose2d point1 = new Pose2d(12,0,0);
    @Override
    public void initialize() {
        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        BeltSubsystem belt = new BeltSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        PusherSubsystem pusher = new PusherSubsystem(hardwareMap, telemetry);

        Pose2d start = new Pose2d(0,0,0);

        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, start), telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);


        // set with your own target poses and headings.
        Pose2d end = new Pose2d(0,0,0);
        Action square = drive.actionBuilder(start)
                .splineToLinearHeading(point1,0)
                // you can add more splines here instead of creating a path2,3,4,etc.
                // make a new distinct path when you need to
                .build();

        Action path2 = drive.actionBuilder(point1).build();


        schedule(new ActionCommand (drive, square));

    }
}