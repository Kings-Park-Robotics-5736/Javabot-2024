package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class IntakeSubsystem extends SubsystemBase {

    private IntakeRollersSubsystem top;
    private IntakeRollersSubsystem bottom;

    public IntakeSubsystem() {

        top = new IntakeRollersSubsystem(Constants.UpperIntakeConstants.kPidValues,
                Constants.UpperIntakeConstants.kFFValues, Constants.UpperIntakeConstants.kDeviceId, "Top");

        bottom = new IntakeRollersSubsystem(Constants.LowerIntakeConstants.kPidValues,
                Constants.LowerIntakeConstants.kFFValues, Constants.LowerIntakeConstants.kDeviceId, "Bottom");

    }

    @Override
    public void periodic() {

    }


    public Command StopIntakeCommand() {
        return Commands.parallel(top.StopIntakeCommand(), bottom.StopIntakeCommand());
    }
    public Command RunIntakeForwardCommand() {
        return Commands.parallel(top.RunIntakeForwardCommand(), bottom.RunIntakeForwardCommand());
    }

    public Command RunIntakeBackwardCommand() {
        return Commands.parallel(top.RunIntakeBackwardCommand(), bottom.RunIntakeBackwardCommand());
    }

}
