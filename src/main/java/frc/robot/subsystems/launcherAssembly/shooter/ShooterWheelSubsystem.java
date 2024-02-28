package frc.robot.subsystems.launcherAssembly.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public class ShooterWheelSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final Boolean m_invert;
    private final String name;
    private double m_forwardSpeed;
    private double m_reverseSpeed;
    private double startTime = 0;

    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    private SimpleMotorFeedforward m_feedforward;
    private TrapezoidProfile profile;


    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_sysIdRoutine;



    public ShooterWheelSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name,
            boolean isInverted, double forwardSpeed, double reverseSpeed) {

        m_motor = new TalonFX(deviceId, "rio");
        TalonFXConfiguration configs = new TalonFXConfiguration();
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        m_invert = isInverted;
        name = _name;
        m_forwardSpeed = forwardSpeed;
        m_reverseSpeed = reverseSpeed;

        configs.Slot0.kP = pidValues.p; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = pidValues.i; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.0;// 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33
                               // = 0.12
                               // volts / Rotation per second
        configs.Slot0.kA = 0.0;// 3.00;
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        m_feedforward = new SimpleMotorFeedforward(ffValues.ks, ffValues.kv, ffValues.ka);

        for (int i = 0; i < 5; ++i) {
            status = m_motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
            else {
                System.out.println("Motor Initialization Failed");
            }

        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the " + name + " Shooter Motor. Restart robot!");
        }
        m_motor.setNeutralMode(NeutralModeValue.Coast);
        m_motor.setInverted(isInverted);

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
                            log.motor(("shooter-" + name))
                                    .voltage(m_appliedVoltage.mut_replace(
                                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                                    .angularPosition(m_distance.mut_replace(m_motor.getPosition().refresh().getValue(),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(m_motor.getVelocity().refresh().getValue(),
                                                    RotationsPerSecond));

                        },
                        this));
    }

    @Override
    public void periodic() {
    }

    public void setNewForwardSpeed(double speed) {
        m_forwardSpeed = speed;
    }

    public void setNewReverseSpeed(double speed) {
        m_reverseSpeed = speed;
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

    public int getSpeedRotationsPerMinute() {
        double rpm = m_motor.getVelocity().refresh().getValue() * 60;
        return (int) rpm;
    }

    public boolean isAtDesiredSpeed() {
        return Math.abs(getSpeedRotationsPerMinute()
                - Constants.ShooterConstants.kDesiredSpeed) < Constants.ShooterConstants.kTolerance;
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunShooterWithMotionProfile() {

        double currTime = Timer.getFPGATimestamp();
        var setpoint = profile.calculate(currTime - startTime);
        double ff = m_feedforward.calculate(setpoint.position / 60); // important, calculate
        // needs rps, not rpm. Hence, / 60
        m_motor.setControl(m_voltageVelocity.withFeedForward(ff).withVelocity(setpoint.position / 60));
    }

    /**
     * Stop the motor
     */
    public void StopShooter() {
        setSpeed(0);
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(6000, 20000),
                new TrapezoidProfile.State(setpoint, 0),
                new TrapezoidProfile.State(getSpeedRotationsPerMinute(), 0));

        startTime = Timer.getFPGATimestamp();

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

    public Command RunShooterForwardCommand(boolean FinishWhenAtTargetSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter forward--------------");
                    InitMotionProfile(m_forwardSpeed);
                },
                () -> {
                    RunShooterWithMotionProfile();
                    SmartDashboard.putNumber(name + " Shooter Fwd vel", getSpeedRotationsPerMinute());
                },
                (interrupted) -> {
                    if (!FinishWhenAtTargetSpeed) {
                        StopShooter();
                    }
                },
                () -> {
                    return FinishWhenAtTargetSpeed && isAtDesiredSpeed();
                }, this);

    }

    public Command RunShooterBackwardCommand(boolean FinishWhenAtTargetSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter Backward--------------");
                    InitMotionProfile(m_reverseSpeed);
                },
                () -> RunShooterWithMotionProfile(),
                (interrupted) -> {
                    if (!FinishWhenAtTargetSpeed) {
                        StopShooter();
                    }
                },
                () -> {
                    return FinishWhenAtTargetSpeed && isAtDesiredSpeed();
                }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}
