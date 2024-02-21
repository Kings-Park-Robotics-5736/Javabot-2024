package frc.robot.subsystems.launcherAssembly.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.utils.Types.PositionType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterWheelSubsystem m_left_wheel;
    private ShooterWheelSubsystem m_right_wheel;

    private double leftForwardSpeed = 0;
    private double rightForwardSpeed = 0;
    private double leftReverseSpeed = 0;
    private double rightReverseSpeed = 0;

    public ShooterSubsystem() {
        m_left_wheel = new ShooterWheelSubsystem(Constants.LeftShooterConstants.kPidValues,
                Constants.LeftShooterConstants.kFFValues, Constants.LeftShooterConstants.kDeviceId, "Left", false,
                Constants.ShooterConstants.kDesiredSpeed, Constants.ShooterConstants.kReverseSpeed);
        m_right_wheel = new ShooterWheelSubsystem(Constants.RightShooterConstants.kPidValues,
                Constants.RightShooterConstants.kFFValues, Constants.RightShooterConstants.kDeviceId, "Right", true,
                Constants.ShooterConstants.kDesiredSpeed, Constants.ShooterConstants.kReverseSpeed);

        leftForwardSpeed = Constants.ShooterConstants.kDesiredSpeed;
        rightForwardSpeed = Constants.ShooterConstants.kDesiredSpeed;
        leftReverseSpeed = Constants.ShooterConstants.kReverseSpeed;
        rightReverseSpeed = Constants.ShooterConstants.kReverseSpeed;
    }

    @Override
    public void periodic() {

        var newLeftForward = SmartDashboard.getNumber("Left Setpoint", 0);
        var newRightForward = SmartDashboard.getNumber("Right Setpoint", 0);

        if (newLeftForward != leftForwardSpeed) {
            leftForwardSpeed = newLeftForward;
            m_left_wheel.setNewForwardSpeed(leftForwardSpeed);
        }

        if (newRightForward != rightForwardSpeed) {
            rightForwardSpeed = newRightForward;
            m_right_wheel.setNewForwardSpeed(rightForwardSpeed);
        }

    }

    /**
     * Stop the shooter, when no command is running. Note this does not work if a
     * command is currently running.
     */
    public Command StopShooterCommand() {
        return Commands.parallel(Commands.runOnce(() -> m_left_wheel.StopShooter()),
                Commands.runOnce(() -> m_right_wheel.StopShooter()));
    }

    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return Commands.parallel(m_left_wheel.RunShooterForwardCommand(FinishWhenAtTargetSpeed),
                m_right_wheel.RunShooterForwardCommand(FinishWhenAtTargetSpeed));
    }

    public Command RunShooterBackwardCommand(boolean FinishWhenAtTargetSpeed) {
        return Commands.parallel(m_left_wheel.RunShooterBackwardCommand(FinishWhenAtTargetSpeed),
                m_right_wheel.RunShooterBackwardCommand(FinishWhenAtTargetSpeed));
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
