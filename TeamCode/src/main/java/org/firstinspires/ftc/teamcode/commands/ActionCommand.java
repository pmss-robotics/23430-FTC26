package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.HashSet;
import java.util.Set;

/**
 * Use this to wrap RR trajectories into commands
 */
public class ActionCommand implements Command {
    private final Action action;
    private final Set<Subsystem> requirements = new HashSet<>();
    private boolean finished = false;

    public ActionCommand(DriveSubsystem drive, Action action) {
        this.action = action;
        requirements.add(drive);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}