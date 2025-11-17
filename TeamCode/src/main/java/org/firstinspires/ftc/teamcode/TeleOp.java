package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.Drawing;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.BeltSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PusherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.States;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    // probably need to change later.
    public static double servoIncrement = 7;
    public static double intakeSlideIncrement = 2;
    public static double servoSpeed = 1;
    public static double driveSpeed = 1;
    public static double fast = 1;
    public static double slow = 0.5;
    public static double rotationSpeed = 1;
    public static double wristStart = 0.5;
//    public static double bucketStart = 0.636;
    public static double outtakeResetPower = 0.4;



    States.Global currentState = States.Global.home;

    GamepadEx driver, tools;
    DriveSubsystem drive;

    VisionSubsystem vision;
    @Override
    public void initialize() {
        FlywheelSubsystem outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        BeltSubsystem belt = new BeltSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        PusherSubsystem pusher = new PusherSubsystem(hardwareMap, telemetry);

        // data sent to telemetry shows up on dashboard and driverGamepad station
        // data sent to the telemetry packet only shows up on the dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(8);
        // GamepadEx wraps gamepad 1 or 2 for easier implementations of more complex key bindings
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx tools = new GamepadEx(gamepad2);
        // The driveSubsystem wraps Roadrunner's MecanumDrive to combine with Commands.
        DriveSubsystem drive = new DriveSubsystem(new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)), telemetry);
        // The driveCommand uses methods defined in the DriveSubsystem to create behaviour.
        // we're passing in methods to get values instead of straight values because it avoids
        // disturbing the structure of the CommandOpMode. The aim is to define bindings in this
        // initialize() method through Commands and these will be looped and acted in the (hidden)
        // run() loop.
        driveSpeed = fast;
        // macros to bring thing up and down
        // intake extenstion
        // outtake macro positions
        DriveCommand driveCommand = new DriveCommand(drive,
                () -> -driver.getLeftX()*driveSpeed,
                () -> driver.getLeftY()*driveSpeed,
                () -> -driver.getRightX()*driveSpeed,
                true);

        /*      try {
            vision = new VisionSubsystem(hardwareMap, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
*/
        // reset everything, probably unnecessary
/*      SequentialCommandGroup returnHome = new SequentialCommandGroup(
                new InstantCommand(() -> intakeSlides.setIntakeSlidesState(States.IntakeExtension.home), intakeSlides),
                new InstantCommand(() -> outtakeSlides.setState(States.OuttakeExtension.home), outtakeSlides),
                new InstantCommand(() -> intake.setWristState(States.Intake.home), intake),
                swapState(States.Global.home)
        );
*/

        // IMU reset
/*        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> drive.drive.localizer.driver.resetPosAndIMU())
        ); */

        //slower driving
        new GamepadButton(driver, GamepadKeys.Button.B).toggleWhenPressed(
                () -> driveSpeed = slow,
                () -> driveSpeed = fast
        );

        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> pusher.moveToTarget(), pusher),
                        new WaitCommand(500),
                        new InstantCommand(() -> pusher.moveToHome(), pusher)
                )
        );

        new GamepadButton(driver, GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(() -> outtake.setPower(-0.7), outtake),
                new InstantCommand(() -> outtake.setPower(0.0), outtake)
        );

        // intake rotation
        new GamepadButton(tools, GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(() -> belt.setPower(0.9), belt),
                new InstantCommand(() -> belt.setPower(0.0), belt)
        );

        new GamepadButton(tools, GamepadKeys.Button.X).toggleWhenPressed(
                new InstantCommand(() -> outtake.setPower(1.0), outtake),
                new InstantCommand(() -> outtake.setPower(0.0), outtake)
        );

        new GamepadButton(tools, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> intake.incrementPosition(-servoIncrement),
                        intake
                ));
        new GamepadButton(tools, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new InstantCommand(
                        () -> intake.incrementPosition(servoIncrement),
                        intake
                ));

        // roller intake rotation
        new GamepadButton(tools, GamepadKeys.Button.A).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(0.5+servoSpeed), intake),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );
        new GamepadButton(tools, GamepadKeys.Button.B).toggleWhenPressed(
                new InstantCommand(() -> intake.setPower(1-servoSpeed), intake),
                new InstantCommand(() -> intake.setPower(0.5), intake)
        );

        // toggle intake slides






        // toggle outtake system

        // TODO transfer system

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
        schedule(new RunCommand(() ->
                outtake.setPower(tools.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)), outtake));


    }

    public InstantCommand swapState(States.Global state) {
        return new InstantCommand(() -> currentState = state);
    }
}
