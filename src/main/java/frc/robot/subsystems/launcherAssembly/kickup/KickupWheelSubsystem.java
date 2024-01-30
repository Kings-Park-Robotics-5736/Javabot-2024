package frc.robot.subsystems.launcherAssembly.kickup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public class KickupWheelSubsystem extends SubsystemBase {


    public KickupWheelSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name) {
    //Todo
    }

    @Override
    public void periodic() {
        //leave blank
    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        //Todo
    }

     /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunIntake(int setpoint) {
        //Todo: hint, look at the same method in IntakeRollerSubsystem
    }

    /**
     * Stop the motor
     */
    public void StopKickup() {
        setSpeed(0);
    }

    public Command RunKickupForwardCommand() {
        return null; //Todo: Replace this line with a proper command
    }

    public Command RunKickupBackwardCommand() {
        return null; //Todo: Replace this line with a proper command
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return null; //Todo: Replace this line with a proper command
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return null; //Todo: Replace this line with a proper command
    }
}
