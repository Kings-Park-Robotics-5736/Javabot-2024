package frc.robot.subsystems.launcherAssembly.arm;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
public class ArmSubsystem extends SubsystemBase {

private final TalonFX m_follower;
private final TalonFX m_leader;
    
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    // do the values defined in velocity voltage ever change??

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_sysIdRoutine;
    

public ArmSubsystem(){
    m_leader = new TalonFX(ArmConstants.kLeaderDeviceId,"rio");
    m_follower = new TalonFX(ArmConstants.kFollowerDeviceId, "rio");
    TalonFXConfiguration configs = new TalonFXConfiguration();
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    m_leader.setNeutralMode(NeutralModeValue.Brake);
    m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
// set m_strictFollower to strict-follow m_leader
// strict followers ignore the leader's invert and use their own



        configs.Slot0.kP = 0.15; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.002; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        // will these P I D and V
        // values ever change?

        for (int i = 0; i < 5; ++i) {
            status = m_leader.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        for (int i = 0; i < 5; ++i) {
            status = m_follower.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }

        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> {
                            // CANNOT use set voltage, it does not work. This normalizes the voltage between
                            // -1 and 0 and 1
                           m_leader.set(volts.in(Volts) / RobotController.getBatteryVoltage());
                        },
                        log -> {
                            log.motor(("Arm"))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_leader.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(m_leader.getPosition().refresh().getValue(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(m_leader.getVelocity().refresh().getValue(),
                                                    RotationsPerSecond));

                        },
                        this));

        // Todo
        // for sysid on this wheel (a falcon 500), use the following motor/encoder
        // commands:
        // to get speed: m_motor.getVelocity().refresh().getValue()
        // to get position: m_encoder.getPosition().refresh().getValue()
        // note unit for speed from the falcon encoder is RPS, not RPM
        // done lesssgoooo

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
        m_leader.set(speed);
    }

    public int getSpeedRotationsPerMinuts() {
        double rpm = m_leader.getVelocity().refresh().getValue() * 60;
        return (int) rpm;
        // please let me know if i shouldn't have done it like this (the conversion)
        // Todo: Replace this line with a proper command, reading the value from the
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
    public void RunShooter(int setpoint) {
        // double ff = m_feedforward.calculate(setpoint / 60); //important, calculate
        // needs rps, not rpm. Hence, / 60
        // m_pidController.setReference(setpoint, CANSparkMax.ControlType.kVelocity, 0,
        // ff, ArbFFUnits.kVoltage);
        // Commented out above is the way its used in the intake subsystem
        // Should be using a feed forward?? or voltage velocity?
       m_leader.setControl(m_voltageVelocity.withVelocity(setpoint));
        // Todo: hint, look at the same method in IntakeRollerSubsystem
    }

    /**
     * Stop the motor
     */
    public void StopArm() {
        setSpeed(0);
    }

    /**
     * 
     * @param FinishWhenAtTargetSpeed When this is set, this command should finish
     *                                once a call to isAtDesiredSpeed returns true.
     *                                When this is false, this command should never
     *                                end.
     * @note When FinishWhenAtTargetSpeed is true, the StopShooter() should not be
     *       called when the command finishes.
     * @return
     */

    /*
     * 
     * 
     * 
     * example from kitbot code
     * 
     * public Command ChargeCommand(boolean isinfinite) {
     * return new FunctionalCommand(
     * () -> {
     * System.out.println("-----------------Charge--------------");
     * 
     * },
     * () -> RunShooter(-6000),
     * (interrupted) -> {
     * if(isinfinite) {StopShooter();}
     * },
     * () -> {return !isinfinite && isShooterAtFullSpeed();});
     * }
     * 
     * 
     * 
     */
    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter forward--------------");
                },
                () -> RunShooter(Constants.ShooterConstants.kDesiredSpeed),
                (interrupted) -> {
                    if (FinishWhenAtTargetSpeed) {
                        StopArm();
                    }
                },
                () -> {
                    return !FinishWhenAtTargetSpeed && isAtDesiredSpeed();
                }, this);

        // return null; // Todo: Replace this line with a proper command
    }

    public Command RunShooterBackwardCommand() {
        return new FunctionalCommand(
                () -> {
                },
                () -> RunShooter(Constants.ShooterConstants.kReverseSpeed),
                (interrupted) -> StopArm(),
                () -> false, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}





