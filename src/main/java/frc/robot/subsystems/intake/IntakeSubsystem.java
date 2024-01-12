package frc.robot.subsystems.intake;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
public class IntakeSubsystem extends SubsystemBase {

    private IntakeRollersSubsystem top;
    private IntakeRollersSubsystem bottom;
    private TimeOfFlight m_tofSensor;

    public IntakeSubsystem() {

        top = new IntakeRollersSubsystem(Constants.UpperIntakeConstants.kPidValues,
                Constants.UpperIntakeConstants.kFFValues, Constants.UpperIntakeConstants.kDeviceId, "Top");

        bottom = new IntakeRollersSubsystem(Constants.LowerIntakeConstants.kPidValues,
                Constants.LowerIntakeConstants.kFFValues, Constants.LowerIntakeConstants.kDeviceId, "Bottom");

  m_tofSensor = new TimeOfFlight(Constants.IntakeConstants.kTofId);
        m_tofSensor.setRangingMode(RangingMode.Short, 40);
                
    }

    @Override
    public void periodic() {
           
         SmartDashboard.putNumber("tof_range",m_tofSensor.getRange());
         SmartDashboard.putBoolean("tof_valid", m_tofSensor.isRangeValid());
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
