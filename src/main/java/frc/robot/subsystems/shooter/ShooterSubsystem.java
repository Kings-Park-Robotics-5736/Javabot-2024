package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class ShooterSubsystem extends SubsystemBase{

    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private final TalonFX m_shooterLeft;
    private final TalonFX m_shooterRight;

    public ShooterSubsystem(){

    m_shooterLeft = new TalonFX(0, "rio");
    m_shooterRight = new TalonFX(1, "rio");


    m_shooterLeft.setNeutralMode(NeutralModeValue.Coast);
    m_shooterRight.setNeutralMode(NeutralModeValue.Coast);



    m_shooterLeft.setInverted(true);
    m_shooterRight.setInverted(false);



    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;



    }

    @Override
    public void periodic(){



    }
    public void runShooter(int rps){
        m_shooterLeft.setControl(m_voltageVelocity.withVelocity(rps));
        m_shooterRight.setControl(m_voltageVelocity.withVelocity(rps));
        SmartDashboard.putNumber("Shooter Left", m_shooterLeft.getVelocity().refresh().getValue());
        SmartDashboard.putNumber("Shooter Left", m_shooterRight.getVelocity().refresh().getValue());
    }
    public void stopShooter(){
        m_shooterRight.setVoltage(0);
        m_shooterLeft.setVoltage(0);

    }

    public Command RunShooterForwardCommand() {
        return new FunctionalCommand(
                () -> {
                    System.out.println("-----------------Starting shooter forward--------------");
                },
                () -> runShooter(6000),
                (interrupted) -> stopShooter(),
                () ->  false, this);
    }
    
    
}
