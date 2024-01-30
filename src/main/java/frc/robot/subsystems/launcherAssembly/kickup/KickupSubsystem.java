package frc.robot.subsystems.launcherAssembly.kickup;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.Types.PositionType;

public class KickupSubsystem extends SubsystemBase {
    
    private KickupWheelSubsystem m_left_wheel;
    private KickupWheelSubsystem m_right_wheel;

    public KickupSubsystem() {
        m_left_wheel = new KickupWheelSubsystem(Constants.LeftKickupConstants.kPidValues,
                Constants.LeftKickupConstants.kFFValues, Constants.LeftKickupConstants.kDeviceId, "Left");
        m_right_wheel = new KickupWheelSubsystem(Constants.RightKickupConstants.kPidValues,
                Constants.RightKickupConstants.kFFValues, Constants.RightKickupConstants.kDeviceId, "Right");
    }


    @Override
    public void periodic() {
        //leave blank
    }


    public Command RunKickupForwardCommand() {
        return Commands.parallel(m_left_wheel.RunKickupForwardCommand(), m_right_wheel.RunKickupForwardCommand());
    }

    public Command RunKickupBackwardCommand() {
        return Commands.parallel(m_left_wheel.RunKickupBackwardCommand(), m_right_wheel.RunKickupBackwardCommand());
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
