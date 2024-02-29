package frc.robot.subsystems.launcherAssembly;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.launcherAssembly.arm.ArmSubsystem;
import frc.robot.subsystems.launcherAssembly.kickup.KickupSubsystem;
import frc.robot.subsystems.launcherAssembly.shooter.ShooterSubsystem;
import frc.robot.utils.Types.PositionType;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class LauncherAssemblySubsystem extends SubsystemBase {

    private final ArmSubsystem m_arm;
    private final ShooterSubsystem m_shooter;
    private final KickupSubsystem m_kickup;
    private final IntakeSubsystem m_intake;

    public LauncherAssemblySubsystem() {
        m_arm = new ArmSubsystem();
        m_shooter = new ShooterSubsystem();
        m_kickup = new KickupSubsystem();
        m_intake = new IntakeSubsystem();
    }

    @Override
    public void periodic() {
        // leave blank
    }

    // Pass Thrus
    public Command RunShooterForwardCommand() {
        return m_shooter.RunShooterForwardCommand(false);
    }

    public Command RunShooterBackwardCommand() {
        return m_shooter.RunShooterBackwardCommand(false);
    }

    public Command RunKickupForwardCommand() {
        return m_kickup.RunKickupForwardCommand();
    }

    public Command RunKickupBackwardCommand() {
        return m_kickup.RunKickupBackwardCommand();
    }
    public Command RunFloorInatakeForwardWithShooterIntakeCommand() {
        /* if position is 0 (or another home position) is should run, else it should not

        if(m_arm.getArmPosition() == 0){
            return new ParallelCommandGroup(m_intake.RunIntakeForwardCommand(), m_shooter.RunShooterForwardCommand(false));}
            else{}
        */
        
            return new ParallelCommandGroup(m_intake.RunIntakeForwardCommand(), m_shooter.RunShooterBackwardCommand(false));
    }
/*
 * 
 * 
 * 
 * 
 * public Command SpinIntakeAndShooterReverseCommand() {
    return new ParallelCommandGroup(
        RunShooterBackwardCommand(),
        RunInatakeForwardCommandAthome()
    );
}
 * 
 * 
 */
    // sysid

    public Command sysIdShooterQuasistatic(SysIdRoutine.Direction direction, PositionType whichMotor) {
        return m_shooter.sysIdQuasistatic(direction, whichMotor);
    }

    public Command sysIdShooterDynamic(SysIdRoutine.Direction direction, PositionType whichMotor) {
        return m_shooter.sysIdDynamic(direction, whichMotor);
    }

    public Command sysIdKickupQuasistatic(SysIdRoutine.Direction direction) {
        return m_kickup.sysIdQuasistatic(direction);
    }

    public Command sysIdKickupDynamic(SysIdRoutine.Direction direction) {
        return m_kickup.sysIdDynamic(direction);
    }
    
    public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction) {
        return m_arm.sysIdQuasistatic(direction);
    }

    public Command sysIdArmDynamic(SysIdRoutine.Direction direction) {
        return m_arm.sysIdDynamic(direction);
    }

    // Composed Commands:

    /**
     * This command will run the shooter, then fire the kickup, then stop the
     * shooter
     * First, it runst the shooter, specifying that it should finish when it reaches
     * the target speed (FinishWhenAtTargetSpeed = true)
     * Then, it runs the kickup, and waits 1 second before continuing. When the 1
     * second elapses, that wins the 'race' condition, causing the
     * RunKickupForwardCommand to be interrupted and stopped.
     * Finally, it stops the shooter.
     * 
     * IMPORTANT - not the parenthesis, and that the raceWith is contained within
     * the andThen, not inline with the andThen.
     * 
     */
    public Command RunShooterAndKickupForwardCommand() {
        return m_shooter.RunShooterForwardCommand(true).andThen(m_kickup.RunKickupForwardCommand()
                .raceWith(Commands.waitSeconds(1)).andThen(m_shooter.StopShooterCommand()));
    }

    public Command RunArmUpManualSpeedCommand(DoubleSupplier getSpeed) {
        return m_arm.RunArmUpManualSpeedCommand(getSpeed);
    }
    public Command RunArmDownManualSpeedCommand(DoubleSupplier getSpeed) {
        return m_arm.RunArmDownManualSpeedCommand(getSpeed);
    }

    public Command RunArmToPositionCommand(double position){
        return m_arm.RunArmToPositionCommand(position);
    }

}
