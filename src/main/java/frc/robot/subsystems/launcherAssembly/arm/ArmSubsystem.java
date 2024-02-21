package frc.robot.subsystems.launcherAssembly.arm;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Types.Limits;

public class ArmSubsystem extends SubsystemBase {

    private final TalonFX m_follower;
    private final TalonFX m_leader;


    private int staleCounter = 0;
    private double lastPosition = 0;

    private ArmFeedforward m_feedforward;
    private TrapezoidProfile m_profile;
    private double startTime = 0;

    private DutyCycleEncoder m_encoder;

    final PositionVoltage m_PositionVoltage = new PositionVoltage(0).withSlot(0);

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_sysIdRoutine;

    public ArmSubsystem() {

        m_leader = new TalonFX(ArmConstants.kLeaderDeviceId, ArmConstants.kCanName);
        m_follower = new TalonFX(ArmConstants.kFollowerDeviceId, ArmConstants.kCanName);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        m_encoder = new DutyCycleEncoder(0);
        m_leader.setNeutralMode(NeutralModeValue.Brake);

        m_feedforward = new ArmFeedforward(ArmConstants.kFFValues.ks, ArmConstants.kFFValues.kg,
                ArmConstants.kFFValues.kv, ArmConstants.kFFValues.ka);
        m_follower.setControl(new Follower(m_leader.getDeviceID(), false));
        // set m_strictFollower to strict-follow m_leader
        // strict followers ignore the leader's invert and use their own

        configs.Slot0.kP = 0.15; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.002; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        // configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333
        // rps per V, 1/8.33 = 0.12
        // volts / Rotation per second
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_leader.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the Leader Arm Motor. Restart robot!");
        }

        status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_follower.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the Leader Arm Motor. Restart robot!");
        }

        m_sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(null, Volts.of(5), null),
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
                                    .angularPosition(m_distance.mut_replace(
                                            m_leader.getPosition().refresh().getValue() / 11.666667,
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(
                                                    m_leader.getVelocity().refresh().getValue() / 11.666667,
                                                    RotationsPerSecond));

                        },
                        this));
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

    public int getSpeedRotationsPerMinute() {
        double rpm = m_leader.getVelocity().refresh().getValue() * 60;
        return (int) rpm;
    }

    public boolean isAtDesiredSpeed() {
        return Math.abs(getSpeedRotationsPerMinute()
                - Constants.ShooterConstants.kDesiredSpeed) < Constants.ShooterConstants.kTolerance;
    }

    public double getArmPosition() {
        return m_encoder.get();
    }

    /**
     * @brief Initializes the motion profile for the elevator
     * @param setpoint of the motor, in absolute rotations
     */
    private void InitMotionProfile(double setpoint) {
        m_profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration),
                new TrapezoidProfile.State(setpoint, 0),
                new TrapezoidProfile.State(getArmPosition(), 0));

        startTime = Timer.getFPGATimestamp();

    }

    public void RunArmToPos() {

        double currTime = Timer.getFPGATimestamp();
        var setpoint = m_profile.calculate(currTime - startTime);
        double ff = m_feedforward.calculate(setpoint.position, setpoint.velocity);

        m_leader.setControl(m_PositionVoltage.withFeedForward(ff).withPosition(setpoint.position));
        SmartDashboard.putNumber("Arm Position", getArmPosition() );
        // Todo: hint, look at the same method in IntakeRollerSubsystem
    }

    /**
     * Stop the motor
     */
    public void StopArm() {
        setSpeed(0);
    }

    /**
     * @brief Checks if the arm has reached its target
     * @return true if the arm has reached its target, false otherwise
     */
    private Boolean armReachedTarget() {

        // check if the arm has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(getArmPosition() - lastPosition) < ArmConstants.kDiffThreshold) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = getArmPosition();

        // calculate the difference between the current position and the motion profile
        // final position
        double delta = Math.abs(getArmPosition() - m_profile.calculate(m_profile.totalTime()).position);

        // we say that the elevator has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < ArmConstants.kDiffThreshold
                || (delta < ArmConstants.kStaleTolerance && staleCounter > ArmConstants.kStaleThreshold);
    }

    private Boolean isFinished() {
        var isFinished = armReachedTarget() || !isWithinLimits();
        return isFinished;
    }

    private Boolean isWithinLimits() {
        return true;
        /* Below needs to be verified with setup
        return (m_leader.get() < 0 && getArmPosition() > ArmConstants.kLimits.low)
                || (m_leader.get() > 0 && getArmPosition() < ArmConstants.kLimits.high);
        */
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

    public Command RunArmToositionCommand(double setpoint) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Arm to position " + setpoint + " --------------");
                    InitMotionProfile(setpoint);
                },
                () -> RunArmToPos(),
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return isFinished();
                }, this);
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Up Starting--------------");
                },
                () -> {
                    setSpeed(getSpeed.getAsDouble());
                    SmartDashboard.putNumber("Arm Position", getArmPosition() );
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return false;
                }, this);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Manual Speed Arm Down Starting--------------");
                },
                () -> {
                    setSpeed(-getSpeed.getAsDouble());
                    SmartDashboard.putNumber("Arm Position", getArmPosition() );
                },
                (interrupted) -> {
                    StopArm();
                },
                () -> {
                    return false;
                }, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction); // Todo: Replace this line with a proper command done
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction); // Todo: Replace this line with a proper command done i think
    }

}
