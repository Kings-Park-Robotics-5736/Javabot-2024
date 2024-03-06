package frc.robot.subsystems.launcherAssembly.kickup;

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
import frc.robot.Constants.kickupConstants;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public class KickupWheelSubsystem extends SubsystemBase {

    private final TalonFX m_motor;
    private final String name;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);
    private SimpleMotorFeedforward m_feedforward;

    /********************************************************
     * SysId variables
     ********************************************************/
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_distance = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final SysIdRoutine m_sysIdRoutine;

    private int counter;

    public KickupWheelSubsystem(PidConstants pidValues, FeedForwardConstants ffValues, byte deviceId, String _name,
            boolean isInverted) {

        m_feedforward = new SimpleMotorFeedforward(ffValues.ks, ffValues.kv, ffValues.ka);

        m_motor = new TalonFX(deviceId, "rio");
        TalonFXConfiguration configs = new TalonFXConfiguration();
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        name = _name;

        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setInverted(false);

        configs.Slot0.kP = 0.15; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.002; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        for (int i = 0; i < 5; ++i) {
            status = m_motor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
            else {
                System.out.println(name + "Motor Initialization Failed");
            }
        }

        if (!status.isOK()) {
            System.out.println("!!!!!ERROR!!!! Could not initialize the " + name + " Kickup Motor. Restart robot!");
        }

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
                            log.motor(("Kickup-" + name))
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
        // leave blank
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
     * @return the speed of the motor in rotations per minute
     */
    public int getSpeedRotationsPerMinutes() {
        double rpm = m_motor.getVelocity().refresh().getValue() * 60;
        return (int) rpm;
    }

    /**
     * 
     * @param setpoint of the motor, in rotations / minute (important, minute not
     *                 seconds)
     */
    public void RunKickup(int setpoint) {
        m_motor.setControl(m_voltageVelocity.withVelocity(setpoint / 60));
        counter ++;
    }

    public void RunKickupHold(){
        System.out.println("Starting Kickup Hold");
        setSpeed(.02);
    }

    /**
     * Stop the motor
     */
    public void StopKickup() {
        setSpeed(0);
    }

    public Command RunKickupForwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Kickup Forward--------------");
                    counter = 0;
                },
                () -> RunKickup(kickupConstants.kForwardSpeed),
                (interrupted) -> StopKickup(),
                () -> false  , this);
    }

    public Command RunKickupBackwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting Kickup Backward--------------");
                },
                () -> RunKickup(kickupConstants.kReverseSpeed),
                (interrupted) -> StopKickup(),
                () -> false, this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
