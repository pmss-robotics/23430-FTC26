package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemNew;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.States;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    // probably need to change later.
    public static double servoIncrement = 7;
    public static double servoSpeed = 1;
    public static double driveSpeed = 1;
    public static double fast = 1;
    public static double slow = 0.5;

    States.Global currentState = States.Global.home; //don't need this?

    GamepadEx driver, tools;
    DriveSubsystem drive;

    VisionSubsystem vision;
    @Override
    public void initialize() {
        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        BeltSubsystem belt = new BeltSubsystem(hardwareMap, telemetry);
        IntakeSubsystemNew intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap, telemetry);

        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        driver = new GamepadEx(gamepad1);
        tools = new GamepadEx(gamepad2);

        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        //drive = new DriveSubsystem(new MecanumDrive(hardwareMap, (Pose2d) blackboard.getOrDefault(End_Pos, new Pose2d(0,0,0))), telemetry);

        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.

        driveSpeed = fast;

        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX()*driveSpeed,
                () -> driver.getLeftY()*driveSpeed,
                () -> -driver.getRightX()*driveSpeed,
                true);

        schedule(driveCommand);

        //DRIVER BUTTONS
        //slower driving
        new GamepadButton(driver, GamepadKeys.Button.B).toggleWhenPressed(
                () -> driveSpeed = slow,
                () -> driveSpeed = fast
        );

        //TOOLS BUTTONS
        //reverse intake & transfer direction
        new GamepadButton(tools, GamepadKeys.Button.A)
                .whileHeld(new RunCommand(() -> intake.setPower(1), intake))
                .whenReleased(new InstantCommand(() -> intake.setPower(0.0))
                );

        //reverse intake direction
        new GamepadButton(tools, GamepadKeys.Button.B).whileHeld(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(-0.4)),
                        new InstantCommand(() -> belt.setPower(-0.6))
                )
        );
        // And separately, you can ensure it stops when released:
        new GamepadButton(tools, GamepadKeys.Button.B).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setPower(0.0)),
                        new InstantCommand(() -> belt.setPower(0.0))
                )
        );

        //turn intake & transfer on
        new GamepadButton(tools, GamepadKeys.Button.X).toggleWhenPressed(
                new InstantCommand(() -> belt.setPower(0.5), belt),
                new InstantCommand(() -> belt.setPower(0.0), belt)
        );

        //toggle kicker
        new GamepadButton(tools, GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(() -> kicker.moveToTarget(), kicker),
                new InstantCommand(() -> kicker.moveToHome(), kicker)
        );

        //turn on belt while pressed
        new GamepadButton(tools, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(() -> belt.setPower(1), belt))
                .whenReleased(new InstantCommand(() -> belt.setPower(0.0))
                );

        //set outtake to 2500 rpm (close shot)
        new GamepadButton(tools, GamepadKeys.Button.DPAD_UP).toggleWhenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(3000), outtake),
                new InstantCommand(() -> outtake.setVelocityRpm(0), outtake)
        );

        //set outtake to 3000 rpm
        new GamepadButton(tools, GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(2500), outtake),
                new InstantCommand(() -> outtake.setVelocityRpm(0), outtake)
        );

        //set outtake speed to 4200 rpm
        new GamepadButton(tools, GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(4080), outtake),
                new InstantCommand(() -> outtake.setPower(0.0), outtake)
        );

        // Driver can snap to locations
/*
        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> Actions.schedule(driveToShootPos))
        );



        Pose2d shootPoint = new Pose2d(22.3, 15.5, Math.toRadians(45));
        Action driveToShootPos = drive.actionBuilder(drive.getPose())
                .splineToLinearHeading(shootPoint, 0)
                .build();

        new GamepadButton(tools, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ActionCommand(drive, driveToShootPos)
                        .andThen(new InstantCommand(() -> schedule(driveCommand)))
        );

 */


        schedule(new RunCommand(() -> {
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d pose = drive.getPose();
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y",pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();


            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }));
    }

}