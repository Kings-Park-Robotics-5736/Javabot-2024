// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Types.DirectionType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.escalator.EscalatorAssemblySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.commands.JoystickCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    private final IntakeSubsystem m_intake = new IntakeSubsystem();

    private final EscalatorAssemblySubsystem m_escalatorAssembly = new EscalatorAssemblySubsystem();

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private void driveWithJoystick(Boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftY(), 0.02))
                * DriveConstants.kMaxSpeedMetersPerSecond;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_driverController.getLeftX(), 0.02))
                * DriveConstants.kMaxSpeedMetersPerSecond;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_driverController.getRightX(), 0.02))
                * DriveConstants.kMaxSpeedMetersPerSecond;

        m_robotDrive.drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> driveWithJoystick(true),
                        m_robotDrive));
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

        new JoystickButton(m_driverController, XboxController.Button.kA.value)
                .onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive));

        new JoystickButton(m_driverController, XboxController.Button.kB.value)
                .toggleOnTrue(m_intake.RunIntakeForwardCommand());

       /* new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(m_escalatorAssembly.RunEscalatorToPositionCommand(0).andThen(JoystickCommands.RumbleControllerTillCancel(m_driverController)));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
                .whileTrue(m_escalatorAssembly.RunEscalatorToPositionCommand(50).andThen(JoystickCommands.RumbleControllerTillCancel(m_driverController)));

        */
        new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(m_escalatorAssembly.ResetElevatorEncoderCommand());

        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(m_escalatorAssembly.RunElevatorToPositionCommand(-61).andThen(JoystickCommands.RumbleControllerTillCancel(m_driverController)));

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
                .whileTrue(m_escalatorAssembly.RunElevatorToPositionCommand(-3).andThen(JoystickCommands.RumbleControllerTillCancel(m_driverController)));

        new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
                .onTrue(m_escalatorAssembly.RunFlipperToPositionCommand(DirectionType.DOWN));

        new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
                .onTrue(m_escalatorAssembly.RunFlipperToPositionCommand(DirectionType.UP));
        /*
        new Trigger(() -> {
            return m_driverController.getRightTriggerAxis() > 0;
        }).whileTrue(m_escalatorAssembly.RunEscalatorManualSpeedCommand(() -> m_driverController.getRightTriggerAxis()));
        new Trigger(() -> {
            return m_driverController.getLeftTriggerAxis() > 0;
        }).whileTrue(m_escalatorAssembly.RunEscalatorManualSpeedCommand(() -> -m_driverController.getLeftTriggerAxis()));
        */
        
        new Trigger(() -> {
                return m_driverController.getRightTriggerAxis() > 0;
        }).and((new JoystickButton(m_driverController, XboxController.Button.kBack.value)).negate()).whileTrue(m_escalatorAssembly.RunElevatorManualSpeedCommand(() -> -m_driverController.getRightTriggerAxis(), false));
        new Trigger(() -> {
        return m_driverController.getLeftTriggerAxis() > 0;
        }).and((new JoystickButton(m_driverController, XboxController.Button.kBack.value)).negate()).whileTrue(m_escalatorAssembly.RunElevatorManualSpeedCommand(() -> m_driverController.getLeftTriggerAxis(), false));


        new Trigger(() -> {
                return m_driverController.getRightTriggerAxis() > 0;
        }).and(new JoystickButton(m_driverController, XboxController.Button.kBack.value)).whileTrue(m_escalatorAssembly.RunElevatorManualSpeedCommand(() -> -m_driverController.getRightTriggerAxis(), true));
        new Trigger(() -> {
        return m_driverController.getLeftTriggerAxis() > 0;
        }).and(new JoystickButton(m_driverController, XboxController.Button.kBack.value)).whileTrue(m_escalatorAssembly.RunElevatorManualSpeedCommand(() -> m_driverController.getLeftTriggerAxis(), true));

        
        /* 
        new Trigger(() -> {
                return m_driverController.getRightTriggerAxis() > 0;
        }).whileTrue(m_escalatorAssembly.RunFlipperManualSpeedCommand(() -> m_driverController.getRightTriggerAxis()));
        new Trigger(() -> {
        return m_driverController.getLeftTriggerAxis() > 0;
        }).whileTrue(m_escalatorAssembly.RunFlipperManualSpeedCommand(() -> -m_driverController.getLeftTriggerAxis()));
        */

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, .5), new Translation2d(2, -.5)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}
