package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class JoystickCommands {

    
    public static Command RumbleControllerTillCancel(GenericHID device) {
        return new FunctionalCommand(
                () -> {},
                () -> device.setRumble(RumbleType.kBothRumble, 1),
                (interrupted) -> device.setRumble(RumbleType.kBothRumble, 0),
                () -> false);
    }
}
