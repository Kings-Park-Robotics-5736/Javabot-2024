package frc.robot.subsystems.climb;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbConstants;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbMotorSubsystem extends SubsystemBase{
    private CANSparkMax m_motor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private SimpleMotorFeedforward m_feedforward;
    private final String name;

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_sysIdRoutine;

    public ClimbMotorSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name,boolean isInverted) {


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

        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            // CANNOT use set voltage, it does not work. This normalizes the voltage between
                            // -1 and 0 and 1
                            m_motor.set(volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor(("Climb-" + name))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(m_encoder.getPosition(), Rotations))
                                    // CRITICAL - encoder returns RPM, we need RPS here. Hence / 60
                                    .angularVelocity(
                                            m_velocity.mut_replace(m_encoder.getVelocity() / 60, RotationsPerSecond));

                        },
                        this));


    }
    
    

    @Override
    public void periodic() {

    }

    /**
     * 
     * Set the speed of the *climb* motor (-1 to 1)
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
    public void RunClimb(int setpoint) {

        double ff = m_feedforward.calculate(setpoint / 60);  //important, calculate needs rps, not rpm. Hence, / 60
        m_pidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity, 0, ff, ArbFFUnits.kVoltage);

        

    }

    public void StopClimb() {
        setSpeed(0);
    }


    public Command RunClimbForwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Climb forward--------------");
                },
                () -> RunClimb(ClimbConstants.kForwardSpeed),
                (interrupted) -> StopClimb(),
                () -> false, this);
    }

    public Command RunClimbBackwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Climb Reverse--------------");
                },
                () -> RunClimb(ClimbConstants.kReverseSpeed),
                (interrupted) -> StopClimb(),
                () -> false, this);
    }



    /***********************************************************
     * SYSID functions
     ***********************************************************/
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

}

