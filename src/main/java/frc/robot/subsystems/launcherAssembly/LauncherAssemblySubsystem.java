package frc.robot.subsystems.launcherAssembly;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.launcherAssembly.arm.ArmSubsystemNEO;
import frc.robot.subsystems.launcherAssembly.kickup.KickupSubsystem;
import frc.robot.subsystems.launcherAssembly.shooter.ShooterSubsystem;
import frc.robot.utils.Types.PositionType;

public class LauncherAssemblySubsystem extends SubsystemBase {

    private final ArmSubsystemNEO m_arm;
    private final ShooterSubsystem m_shooter;
    private final KickupSubsystem m_kickup;

    public LauncherAssemblySubsystem(DriveSubsystem drive) {
        m_arm = new ArmSubsystemNEO();
        m_shooter = new ShooterSubsystem(drive);
        m_kickup = new KickupSubsystem();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Arm Has Note", ArmContainsNote());
        
    }

    // Pass Thrus

    /***************************************
     * SHOOTER
     *****************************************/
    public Command RunShooterForwardCommand() {
        return m_shooter.RunShooterForwardCommand(false);
    }

    public Command RunShooterForAmp() {
        return m_shooter.RunShooterForwardForAmp();
    }

    public Command RunShooterForwardForScorpion() {
        return (m_shooter.RunShooterForwardForScorpion(true).handleInterrupt(() -> m_shooter.StopShooter()))
                .andThen(RunKickupForwardCommandWithTimer().handleInterrupt(() -> m_shooter.StopShooter()))
                .andThen(m_shooter.StopShooterCommand());
    }

    public Command MoveToScorpionAndShootCommand() {
        return (m_shooter.RunShooterForwardForScorpion(true).alongWith(runArmToScorpionPositionCommand()))
                .andThen(RunKickupForwardCommandWithTimer().handleInterrupt(() -> m_shooter.StopShooter()))
                .andThen(m_shooter.StopShooterCommand());
    }

    public Command RunShooterForwardIdle() {
        return m_shooter.RunShooterForwardIdle();
    }

    public Command RunShooterBackwardCommand() {
        return m_shooter.RunShooterBackwardCommand(false);
    }

    public Command StopShooterCommand() {
        return m_shooter.StopShooterCommand();
    }

    public Command SpoolShooterCommand() {
        return m_shooter.SpoolShooterCommand();
    }

    public boolean shooterAtSpeed(){
        return m_shooter.shooterAtSpeed();
    }

    public boolean shooterAtSpeedOrFaster(){
        return m_shooter.shooterAtSpeedOrFaster();
    }

    /*****************************
     * Kickup
     ******************************/
    public Command RunKickupForwardCommand() {
        return m_kickup.RunKickupForwardCommand();
    }

    public Command RunKickupHoldCommand() {
        return m_kickup.RunKickupHoldCommand();
    }

    public void RunKickupHold() {
        m_kickup.RunKickupHold();
    }

    public Command RunKickupForwardCommandWithTimer() {
        return (m_kickup.RunKickupForwardCommand().withTimeout(0.75)).until(() -> !m_arm.hasNote())
                .andThen(Commands.waitSeconds(0.25));
    }

    public Command RunKickupBackwardCommand() {
        return m_kickup.RunKickupBackwardCommand();
    }

    


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
        return (m_shooter.RunShooterForwardCommand(true).handleInterrupt(() -> m_shooter.StopShooter()))
                .andThen(RunKickupForwardCommandWithTimer().handleInterrupt(() -> m_shooter.StopShooter()))
                .andThen(m_shooter.StopShooterCommand());
    }

    public Command ShootCuzIToldYouYouAreReady(){
        return (RunKickupForwardCommandWithTimer().handleInterrupt(() -> m_shooter.StopShooter())).andThen(m_shooter.StopShooterCommand());
    }



    /*******************************
     * Arm Commands
     */
    public Command RunArmUpManualSpeedCommand(DoubleSupplier getSpeed) {
        return m_arm.RunArmUpManualSpeedCommand(getSpeed);
    }

    public Command RunArmDownManualSpeedCommand(DoubleSupplier getSpeed) {
        return m_arm.RunArmDownManualSpeedCommand(getSpeed);
    }

    public Command RunArmToPositionCommand(double position) {
        return m_arm.RunArmToPositionCommand(position);
    }

    public Command RunArmToIntakePositionCommand() {
        return RunArmToPositionCommand(ArmConstants.intakeAngle).until(() -> m_arm.armIsDown());
    }

    public Command runArmToAmpPositionCommand() {
        return m_arm.RunArmToPositionCommand(ArmConstants.ampAngle);
    }

    public Command runArmToScorpionPositionCommand() {
        return m_arm.RunArmToPositionCommand(ArmConstants.scorpionAngle);
    }

    public Command runArmToTrapCommand() {
        return m_arm.RunArmToPositionCommand(ArmConstants.TrapAngle);
    }

    public Command runArmToFixedAngleCommand(){
        return m_arm.RunArmToPositionCommand(ArmConstants.FixedAngle);
    }

    public Command RunArmToAutoPositionCommand(DriveSubsystem robotDrive) {
        return m_arm.RunArmToAutoPositionCommand(robotDrive, true);
    }

    public Command RunArmToAutoPositionCommandContinuous(DriveSubsystem robotDrive) {
        return m_arm.RunArmToAutoPositionCommand(robotDrive, false);
    }

    public boolean ArmContainsNote() {
        return m_arm.hasNote();
    }

    public boolean armIsDown() {
        return m_arm.armIsDown();
    }
    public boolean armIsAtPosition(){
        return m_arm.armReachedTarget();
    }

    public Command UpdateArmAngleManually(double diff) {
        return Commands.runOnce(() -> m_arm.UpdateAngleManually(diff));

    }

    public void resetArm() {
        m_arm.resetAfterDisable();
    }

    public void ResetArmEncoder(){
        m_arm.resetArmHomePosition();
    }

}
