package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.CenterToTargetCommandLimelight;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.PathPlanFromDynamicStartCommand;
import frc.robot.field.ScoringPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiCamera;

public class CommandMap {

    private final HashMap<String, Command> eventMap;

    public CommandMap(DriveSubsystem robotDrive, PiCamera picam, Limelight limelight) {

        eventMap = new HashMap<>();

        eventMap.put("Forward1", new DriveDistanceCommand(robotDrive, 1));
        eventMap.put("Forward0.5", new DriveDistanceCommand(robotDrive, 0.5));
        eventMap.put("GrabTarget", new DriveToTargetCommand(robotDrive, picam, 2.25, 1));

        eventMap.put("LimelightVisionPipeline", Commands.runOnce(() -> limelight.setReflectivePipeline()));
        eventMap.put("ForceStop", Commands.runOnce(() -> robotDrive.forceStop()));
    }

    /**
     * @brief Returns the command associated with the given command name. If no
     *        command is associated with the given name, returns a command that 
     *        does nothing.
     * @note
     *        By returning command that does nothing (instead of typical null), we
     *        can help prevent NullPointerExceptions / crashes.
     * 
     * @param commandName the name of the command to get
     * @return the command associated with the given command name
     */
    public Command getCommand(String commandName) {
        return eventMap.getOrDefault(commandName, Commands.runOnce(() -> {}));
    }

    public HashMap<String, Command> getRawMap() {
        return eventMap;
    }

}
