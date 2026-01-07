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
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {

    public static double driveSpeed = 1;
    public static double fast = 1;
    public static double slow = 0.5;

    //define controllers
    GamepadEx driver, tools;
    DriveSubsystem drive;

    @Override
    public void initialize(){
        //initialize subsystems
        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        TransferSubsystem belt = new TransferSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap, telemetry);

        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);

        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        driver = new GamepadEx(gamepad1);
        tools = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);

        driveSpeed = fast;

        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX()*driveSpeed,
                () -> driver.getLeftY()*driveSpeed,
                () -> -driver.getRightX()*driveSpeed,
                true);

        //DRIVER BUTTONS!!

        //adjust driveSpeed
        new GamepadButton(driver, GamepadKeys.Button.B).toggleWhenPressed(
                () -> driveSpeed = slow,
                () -> driveSpeed = fast
        );

        //reset position & heading
        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(
                () -> drive.drive.localizer.setPose(new Pose2d(0,0,0))
        );

        //TOOLS BUTTONS!!

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
                new InstantCommand(() -> outtake.setVelocityRpm(2900), outtake),
                new InstantCommand(() -> outtake.setVelocityRpm(0), outtake)
        );

        //set outtake to 3000 rpm
        new GamepadButton(tools, GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(2800), outtake),
                new InstantCommand(() -> outtake.setVelocityRpm(0), outtake)
        );

        //set outtake speed to 4200 rpm
        new GamepadButton(tools, GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(
                new InstantCommand(() -> outtake.setVelocityRpm(3400), outtake),
                new InstantCommand(() -> outtake.setPower(0.0), outtake)
        );

        //schedule all commands here!
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
        schedule(driveCommand);

    }

}
