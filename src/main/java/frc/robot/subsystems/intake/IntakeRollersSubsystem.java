package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;




import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public class IntakeRollersSubsystem extends SubsystemBase {

    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private SimpleMotorFeedforward m_feedforward;
    private final String name;
    private boolean m_stop;

    public IntakeRollersSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name) {

        m_motor = new CANSparkMax(deviceId, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        name = _name;

        
        m_pidController.setP(pidValues.p);
        m_pidController.setI(pidValues.i);
        m_pidController.setD(pidValues.d);
        m_pidController.setIZone(0);
        m_pidController.setFF(0);
        m_pidController.setOutputRange(-1, 1);
        m_feedforward = new SimpleMotorFeedforward(ffValues.ks, ffValues.kv, ffValues.ka);
      
        

    }

    @Override
    public void periodic() {

     
         


    }

    /**
     * 
     * Set the speed of the intake motor (-1 to 1)
     * 
     * @param speed
     */
    private void setSpeed(double speed) {
        m_motor.set(speed);
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunIntake(int setpoint) {

        double ff = m_feedforward.calculate(setpoint / 60);
        m_pidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity, 0, ff, ArbFFUnits.kVoltage);

    }

    public void StopIntake() {
        m_stop = true;
        setSpeed(0);
    }
    public boolean withinRange(){
       /* if(m_tofSensor.getRange() < 5){
            return true;
        }
        else{
            return false;
        }
        */
        return false;
    }

    public Command StopIntakeCommand() {
        return this.runOnce(() -> StopIntake());
    }

    public Command RunIntakeForwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting intake forward--------------");
                    m_stop = false;
                },
                () -> RunIntake(IntakeConstants.kForwardSpeed),
                (interrupted) -> StopIntake(),
                () ->  withinRange(), this);
    }

    public Command RunIntakeBackwardCommand() {
        return new FunctionalCommand(
                () -> {
                    m_stop = false;
                },
                () -> RunIntake(IntakeConstants.kReverseSpeed),
                (interrupted) -> StopIntake(),
                () -> m_stop, this);
    }

}


