package frc.robot.subsystems.launcherAssembly.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;
import frc.robot.Constants;

public class ShooterWheelSubsystem extends SubsystemBase {

    public ShooterWheelSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name) {
        // Todo
        // for sysid on this wheel (a falcon 500), use the following motor/encoder
        // commands:
        // to get speed: m_motor.getVelocity().refresh().getValue()
        // to get position: m_encoder.getPosition().refresh().getValue() 
        //note unit for speed from the falcon encoder is RPS, not RPM

    }

    @Override
    public void periodic() {
        // leave blank
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        // Todo
    }

    public int getSpeedRotationsPerMinuts() {
        return 0; // Todo: Replace this line with a proper command, reading the value from the
                  // encoder. PAY ATTENTION TO UNITS!
    }

    public boolean isAtDesiredSpeed() {
        return Math.abs(getSpeedRotationsPerMinuts()
                - Constants.ShooterConstants.kDesiredSpeed) < Constants.ShooterConstants.kTolerance;
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunIntake(int setpoint) {
        // Todo: hint, look at the same method in IntakeRollerSubsystem
    }

    /**
     * Stop the motor
     */
    public void StopShooter() {
        setSpeed(0);
    }

    /**
     * 
     * @param FinishWhenAtTargetSpeed When this is set, this command should finish once a call to isAtDesiredSpeed returns true. When this is false, this command should never end.
     * @note When FinishWhenAtTargetSpeed is true, the StopShooter() should not be called when the command finishes.
     * @return
     */
    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return null; // Todo: Replace this line with a proper command
    }

    public Command RunShooterBackwardCommand() {
        return null; // Todo: Replace this line with a proper command
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return null; // Todo: Replace this line with a proper command
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return null; // Todo: Replace this line with a proper command
    }
}
