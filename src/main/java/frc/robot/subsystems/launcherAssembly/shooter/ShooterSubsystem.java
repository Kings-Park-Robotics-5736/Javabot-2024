package frc.robot.subsystems.launcherAssembly.shooter;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utils.Types.PositionType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterWheelSubsystem m_left_wheel;
    private ShooterWheelSubsystem m_right_wheel;

    public ShooterSubsystem() {
        m_left_wheel = new ShooterWheelSubsystem(Constants.LeftShooterConstants.kPidValues,
                Constants.LeftShooterConstants.kFFValues, Constants.LeftShooterConstants.kDeviceId, "Left",false);
        m_right_wheel = new ShooterWheelSubsystem(Constants.RightShooterConstants.kPidValues,
                Constants.RightShooterConstants.kFFValues, Constants.RightShooterConstants.kDeviceId, "Right",false);
    }

    @Override
    public void periodic() {
        //leave blank
    }

    /**
     * Stop the shooter, when no command is running. Note this does not work if a command is currently running.
     */
    public Command StopShooterCommand(){
        return Commands.parallel(Commands.runOnce(()->m_left_wheel.StopShooter()), Commands.runOnce(()->m_right_wheel.StopShooter()));
    }

    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return Commands.parallel(m_left_wheel.RunShooterForwardCommand(FinishWhenAtTargetSpeed), m_right_wheel.RunShooterForwardCommand(FinishWhenAtTargetSpeed));
    }

    public Command RunShooterBackwardCommand() {
        return Commands.parallel(m_left_wheel.RunShooterBackwardCommand(), m_right_wheel.RunShooterBackwardCommand());
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, PositionType whichSide) {
        if (whichSide == PositionType.LEFT) {
            return m_left_wheel.sysIdQuasistatic(direction);
        } else {
            return m_right_wheel.sysIdQuasistatic(direction);
        }

    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, PositionType whichSide) {
         if (whichSide == PositionType.LEFT) {
            return m_left_wheel.sysIdDynamic(direction);
        } else {
            return m_right_wheel.sysIdDynamic(direction);
        }
    }
}
