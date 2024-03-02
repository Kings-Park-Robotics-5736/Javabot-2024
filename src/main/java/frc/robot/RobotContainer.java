// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.JoystickCommandsFactory;
import frc.robot.commands.RobotCommandsFactory;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.commands.drive.DriveDistanceCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcherAssembly.LauncherAssemblySubsystem;
import frc.robot.utils.Types.PositionType;
import frc.robot.utils.Types.SysidMechanism;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.LEDMode;
import frc.robot.vision.PiCamera;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        // The robot's subsystems

        private final SysidMechanism enabledSysid = SysidMechanism.NONE;

        private final PiCamera m_picam = new PiCamera();
        public Limelight m_limelight = new Limelight("limelight");
        public Limelight m_limelight_side = new Limelight("limelight-side");

        private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_limelight, null);// use only 1 limelight for
                                                                                          // driving now since we dont
                                                                                          // have great measurements
                                                                                          // m_limelight_side);

        private final IntakeSubsystem m_intake = new IntakeSubsystem();
        private final LauncherAssemblySubsystem m_Launcher = new LauncherAssemblySubsystem();
        private final PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
        private final SendableChooser<Command> autoChooser;

        private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
        private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

        XboxController m_driverController = new XboxController(IOConstants.kDriverControllerPort);
        XboxController m_actionController = new XboxController(IOConstants.kActionControllerPort);

        private void driveWithJoystick(Boolean fieldRelative) {
                // Get the x speed. We are inverting this because Xbox controllers return
                // negative values when we push forward.
                var xSpeed = -m_xspeedLimiter
                                .calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
                                * DriveConstants.kMaxSpeedMetersPerSecond;

                // Get the y speed or sideways/strafe speed. We are inverting this because
                // we want a positive value when we pull to the left. Xbox controllers
                // return positive values when you pull to the right by default.
                var ySpeed = -m_yspeedLimiter
                                .calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
                                * DriveConstants.kMaxSpeedMetersPerSecond;

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                        xSpeed = -xSpeed;
                        ySpeed = -ySpeed;
                }

                // Get the rate of angular rotation. We are inverting this because we want a
                // positive value when we pull to the left (remember, CCW is positive in
                // mathematics). Xbox controllers return positive values when you pull to
                // the right by default.
                final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.02))
                                * DriveConstants.kMaxRotationSpeedMetersPerSecond;

                m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative, true);
        }

        private void InitializeNamedCommands() {
                NamedCommands.registerCommand("Forward1", new DriveDistanceCommand(m_robotDrive, 1));
                NamedCommands.registerCommand("Forward0.5", new DriveDistanceCommand(m_robotDrive, 0.5));
                NamedCommands.registerCommand("GrabTarget", new DriveToTargetCommand(m_robotDrive, m_picam, 2.25, 1));
                NamedCommands.registerCommand("ForceStop", Commands.runOnce(() -> m_robotDrive.forceStop()));
                NamedCommands.registerCommand("RunIntake", m_intake.RunIntakeForwardCommand());
                NamedCommands.registerCommand("DriveToNote", new DriveToTargetCommand(m_robotDrive, m_picam, 1, 1));
                NamedCommands.registerCommand("DriveToNoteWithIntake",RobotCommandsFactory.DriveToTargetWithIntake(m_robotDrive, m_intake, m_Launcher, m_picam, 2.0, 3.0));
                NamedCommands.registerCommand("StartShooter", m_Launcher.RunShooterForwardCommand());
                NamedCommands.registerCommand("ShootWhenReady", m_Launcher.RunShooterAndKickupForwardCommand());
                NamedCommands.registerCommand("ArmToAutoAngle", m_Launcher.RunArmToAutoPositionCommand(m_robotDrive));
                NamedCommands.registerCommand("CenterToTarget", new CenterToGoalCommand(m_robotDrive, false));
                NamedCommands.registerCommand("ArmToIntakePose", m_Launcher.RunArmToIntakePositionCommand());

                NamedCommands.registerCommand("ShootScorpion", m_Launcher.RunShooterForwardForScorpion());
                NamedCommands.registerCommand("ArmToScorpionAngle", m_Launcher.runArmToScorpionPositionCommand());
                NamedCommands.registerCommand("MoveToScorpionAndShoot", m_Launcher.MoveToScorpionAndShootCommand());
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                SmartDashboard.putNumber("Left Setpoint", 0);
                SmartDashboard.putNumber("Right Setpoint", 0);

                InitializeNamedCommands(); // must do this first

                // Configure the button bindings
                switch (enabledSysid) {

                        case INTAKE_TOP:
                        case INTAKE_BOTTOM:
                                configureButtonBindingsIntakeSysID();
                                break;
                        case DRIVE:
                                configureButtonBindingsDriveSysID();
                                break;
                        case SHOOTER_LEFT:
                                configureButtonBindingsShooterLeftSysID();
                                break;
                        case SHOOTER_RIGHT:
                                configureButtonBindingsShooterRightSysID();
                                break;
                        case KICKUP_RIGHT:
                                configButtonBindingsKickupRightSysID();
                                break;
                        case ARM:
                                configButtonBindingsArmSysID();
                                break;
                        case NONE:
                        default:
                                configureButtonBindings();
                                // Configure default commands
                                PDH.setSwitchableChannel(true);
                                // Enable LEDS
                                // will this happen when we turn on/push code?
                                // or on enable
                                m_robotDrive.setDefaultCommand(
                                                // The left stick controls translation of the robot.
                                                // Turning is controlled by the X axis of the right stick.
                                                new RunCommand(
                                                                () -> driveWithJoystick(true),
                                                                m_robotDrive));
                                break;

                }

                // Set limelight LED to follow pipeline on startup
                m_limelight.setLEDMode(LEDMode.PIPELINE);

                autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
                SmartDashboard.putData("Auto Mode", autoChooser);

        }

        public void resetArm(){
                m_Launcher.resetArm();
        }
        private void configureButtonBindingsDriveSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        private void configureButtonBindingsIntakeSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward,
                                                PositionType.TOP)
                                                .alongWith(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward,
                                                                PositionType.BOTTOM)));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse,
                                                PositionType.TOP)
                                                .alongWith(m_intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse,
                                                                PositionType.BOTTOM)));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_intake.sysIdDynamic(SysIdRoutine.Direction.kForward, PositionType.TOP)
                                                .alongWith(m_intake.sysIdDynamic(SysIdRoutine.Direction.kForward,
                                                                PositionType.BOTTOM)));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_intake.sysIdDynamic(SysIdRoutine.Direction.kReverse, PositionType.TOP)
                                                .alongWith(m_intake.sysIdDynamic(SysIdRoutine.Direction.kReverse,
                                                                PositionType.BOTTOM)));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_intake.RunIntakeForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_intake.RunIntakeBackwardCommand());
        }

        private void configureButtonBindingsShooterLeftSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kForward,
                                                PositionType.LEFT));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kReverse,
                                                PositionType.LEFT));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kForward,
                                                PositionType.LEFT));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kReverse,
                                                PositionType.LEFT));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterBackwardCommand());
        }

        private void configureButtonBindingsShooterRightSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kForward,
                                                PositionType.RIGHT));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdShooterQuasistatic(SysIdRoutine.Direction.kReverse,
                                                PositionType.RIGHT));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kForward,
                                                PositionType.RIGHT));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdShooterDynamic(SysIdRoutine.Direction.kReverse,
                                                PositionType.RIGHT));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_Launcher.RunShooterBackwardCommand());
        }

        private void configButtonBindingsKickupRightSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdKickupQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdKickupQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdKickupDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdKickupDynamic(SysIdRoutine.Direction.kReverse));


                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .toggleOnTrue(m_Launcher.RunKickupForwardCommand());

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(m_Launcher.RunKickupBackwardCommand());
        }

        private void configButtonBindingsArmSysID() {
                new JoystickButton(m_driverController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.sysIdArmQuasistatic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.sysIdArmQuasistatic(SysIdRoutine.Direction.kReverse));
                new JoystickButton(m_driverController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.sysIdArmDynamic(SysIdRoutine.Direction.kForward));
                new JoystickButton(m_driverController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.sysIdArmDynamic(SysIdRoutine.Direction.kReverse));

                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(m_Launcher.RunArmToPositionCommand(0));

                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_Launcher.RunArmToPositionCommand(Math.toRadians(-25)));


                 new Trigger(() -> {
                        return m_driverController.getRightTriggerAxis() > 0;
                }).whileTrue(m_Launcher.RunArmUpManualSpeedCommand(() -> m_driverController.getRightTriggerAxis()));


                new Trigger(() -> {
                        return m_driverController.getLeftTriggerAxis() > 0;
                }).whileTrue(m_Launcher.RunArmDownManualSpeedCommand(() -> -m_driverController.getLeftTriggerAxis()));
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                /*
                 * // BACK BUTTON: zero out the heading. Note, with vision pose, this should not
                 * be
                 * // used anymore.
                 * new JoystickButton(m_driverController, XboxController.Button.kBack.value)
                 * .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive));
                 * 
                 * // X BUTTON: drive forward 1 meter
                 * // new JoystickButton(m_driverController, XboxController.Button.kX.value)
                 * // .whileTrue(new DriveDistanceCommand(m_robotDrive, 1));
                 * 
                 * // Y BUTTON: center on a game piece
                 * new JoystickButton(m_driverController, XboxController.Button.kY.value)
                 * .whileTrue(new CenterToTargetCommandPiCam(m_robotDrive, m_picam, true));
                 * 
                 * // new JoystickButton(m_driverController, XboxController.Button.kA.value)
                 * // .whileTrue(new CenterToGoalCommand(m_robotDrive,m_limelight, true));
                 * 
                 * // B BUTTON: Drive to a target, and stop when you reach it.
                 * new JoystickButton(m_driverController, XboxController.Button.kB.value)
                 * .whileTrue(new DriveToTargetCommand(m_robotDrive, m_picam, 1, 4));
                 * 
                 * new JoystickButton(m_actionController, XboxController.Button.kB.value)
                 * .toggleOnTrue(m_intake.RunIntakeForwardCommand());
                 * 
                 * new JoystickButton(m_actionController, XboxController.Button.kX.value)
                 * .toggleOnTrue(m_intake.RunIntakeBackwardCommand());
                 * 
                 * new JoystickButton(m_actionController, XboxController.Button.kB.value)
                 * .toggleOnTrue(m_intake.RunIntakeForwardCommand());
                 * 
                 * new JoystickButton(m_actionController, XboxController.Button.kX.value)
                 * .toggleOnTrue(m_intake.RunIntakeBackwardCommand());
                 * 
                 */

                /*
                 * new JoystickButton(m_actionController, XboxController.Button.kX.value)
                 * .whileTrue(m_Launcher.RunShooterBackwardCommand());
                 * 
                 * new JoystickButton(m_actionController, XboxController.Button.kA.value)
                 * .whileTrue(m_Launcher.RunKickupForwardCommand());
                 */



                new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).toggleOnTrue(m_intake.RunIntakeForwardCommand());
                new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).toggleOnTrue(m_intake.RunIntakeBackwardCommand());

                 new JoystickButton(m_driverController, XboxController.Button.kA.value)
                 .whileTrue(new CenterToGoalCommand(m_robotDrive, true));

                  new JoystickButton(m_driverController, XboxController.Button.kB.value)
                 .whileTrue( RobotCommandsFactory.DriveToTargetWithIntake(m_robotDrive, m_intake, m_Launcher, m_picam, 2.0, 3.0));


                new Trigger(() -> {
                        return m_actionController.getRightTriggerAxis() > 0;
                }).whileTrue(m_Launcher.RunShooterForwardCommand());


                new Trigger(() -> {
                        return m_actionController.getLeftTriggerAxis() > 0;
                }).whileTrue(m_Launcher.RunArmDownManualSpeedCommand(() -> -m_actionController.getLeftTriggerAxis()/5));


                new Trigger(() -> {
                        return m_actionController.getRightY() > 0.15 || m_actionController.getRightY() < -0.15;
                }).whileTrue(m_Launcher.RunArmUpManualSpeedCommand(() -> m_actionController.getRightY()/5));



                new JoystickButton(m_actionController, XboxController.Button.kLeftBumper.value)
                                .whileTrue(RobotCommandsFactory.RunFloorIntakeWithArmPosition(m_intake, m_Launcher));

                new JoystickButton(m_actionController, XboxController.Button.kBack.value)
                  .whileTrue((m_intake.RunIntakeBackwardCommand().alongWith(m_Launcher.RunShooterForwardCommand()).alongWith(m_Launcher.RunKickupForwardCommand())));

                new JoystickButton(m_actionController, XboxController.Button.kRightBumper.value)
                                .whileTrue(m_Launcher.RunShooterAndKickupForwardCommand().andThen(JoystickCommandsFactory
                                .RumbleControllerTillCancel(m_actionController)));
                

                new JoystickButton(m_actionController, XboxController.Button.kX.value)
                                .whileTrue(m_Launcher.runArmToAmpPositionCommand().andThen(JoystickCommandsFactory
                                .RumbleControllerTillCancel(m_actionController)));

                new JoystickButton(m_actionController, XboxController.Button.kB.value)
                                .whileTrue(m_Launcher.RunArmToPositionCommand(Math.toRadians(-45)).andThen(JoystickCommandsFactory
                                .RumbleControllerTillCancel(m_actionController)));

                new JoystickButton(m_actionController, XboxController.Button.kA.value)
                                .whileTrue(m_Launcher.RunArmToIntakePositionCommand().andThen(JoystickCommandsFactory
                                .RumbleControllerTillCancel(m_actionController)));
                
                new JoystickButton(m_actionController, XboxController.Button.kY.value)
                                .whileTrue(m_Launcher.RunArmToAutoPositionCommandContinuous(m_robotDrive).andThen(JoystickCommandsFactory
                                .RumbleControllerTillCancel(m_actionController)));

                // the POV hat works differently;
                // it returns a value from 0-360, where 0 is up, 90 is right, 180 is down, and
                // 270 is left.
                // you can get values in between, so we need to define a range.
                new Trigger(() -> {
                        return m_actionController.getPOV() > 340 || (m_actionController.getPOV() < 20 && m_actionController.getPOV()>=0);
                }).onTrue(m_Launcher.UpdateArmAngleManually(Math.toRadians(-1)));

                new Trigger(() -> {
                        return m_actionController.getPOV() < 200 && m_actionController.getPOV() < 160;
                }).onTrue(m_Launcher.UpdateArmAngleManually(Math.toRadians(1)));

                new Trigger(() -> {
                        return m_actionController.getPOV() > 20 && m_actionController.getPOV() < 180;
                }).whileTrue(Commands.parallel(m_Launcher.RunShooterForAmp(), m_Launcher.RunKickupForwardCommand()));

                new Trigger(() -> {
                        return m_actionController.getPOV() < 180 && m_actionController.getPOV() < 360;
                }).onTrue(m_Launcher.RunKickupForwardCommand());
                

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                SmartDashboard.putString("Auto Running", autoChooser.getSelected().getName());
                return autoChooser.getSelected();
        }

        public void publishAuto() {
                SmartDashboard.putString("Auto Chosen", autoChooser.getSelected().getName());
                SmartDashboard.putNumber("Pi Theta", m_picam.getPiCamAngle());
        }

}
