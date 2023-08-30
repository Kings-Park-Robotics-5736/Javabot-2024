// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  /****************************************************
   * Define the 4 swerve modules
   ****************************************************/
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      DriveConstants.kFrontLeftDriveReversed,
      DriveConstants.kFrontLeftAngleOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kRearLeftTurningEncoderReversed,
      DriveConstants.kRearLeftDriveReversed,
      DriveConstants.kBackLeftAngleOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kFrontRightTurningEncoderReversed,
      DriveConstants.kFrontRightDriveReversed,
      DriveConstants.kFrontRightAngleOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kCanName,
      DriveConstants.kRearRightTurningEncoderReversed,
      DriveConstants.kRearRightDriveReversed,
      DriveConstants.kBackRightAngleOffset);

  // The gyro sensor
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(0, "Canivore");

  private final double kDt = 0.02;

  private final DoubleSubscriber ySub;

  /**
   * @brief This is a flag to lockout the joystick control of the robot.
   *        This is used when the robot is running an auto-command.
   *        It is needed, or else the joystick will fight the auto-command with
   *        its default 0's.
   */
  private boolean m_joystickLockoutTranslate;
  private boolean m_joystickLockoutRotate;
  private double m_rotateLockoutValue;
  private double m_transXLockoutValue;
  private double m_transYLockoutValue;

  private boolean m_gotTarget;
  private int m_gotTargetCounter;
  private Pose2d m_startingPosition;

  // Control the motion profile for the auto-commands for driving. This is kind-of
  // like a path following
  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
      DriveConstants.kMaxSpeedMetersPerSecond, 2);
  private final ProfiledPIDController m_controller_x = new ProfiledPIDController(.5, 0.1, 0.000, m_constraints, kDt);
  private final ProfiledPIDController m_controller_y = new ProfiledPIDController(.5, 0.1, 0.000, m_constraints, kDt);
  private final ProfiledPIDController m_controller_theta = new ProfiledPIDController(1, 0, 0.000, m_constraints, kDt);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    m_joystickLockoutTranslate = false;
    m_joystickLockoutRotate = false;

    m_transXLockoutValue = 0;
    m_transYLockoutValue = 0;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "datatable"
    NetworkTable datatable = inst.getTable("SmartDashboard");

    // subscribe to the topic in "datatable" called "Y"
    ySub = datatable.getDoubleTopic("anglex").subscribe(0.0);

    // m_controller_theta.enableContinuousInput(-Math.PI, Math.PI);
    m_controller_theta.setTolerance(0.01);

  }

  /**
   * Update odometry from the current positions and angles.
   */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("Rotation", m_gyro.getRotation2d().getDegrees());

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();

    // double value = ySub.get();

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    drive(xSpeed, ySpeed, rot, fieldRelative, false);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean joystick) {

    if ((joystick && !m_joystickLockoutTranslate) || !joystick) {

      if (m_joystickLockoutRotate && joystick) {
        rot = m_rotateLockoutValue;
        m_transXLockoutValue = xSpeed;
        m_transYLockoutValue = ySpeed;
        fieldRelative = false;
      } else if (m_joystickLockoutRotate && !joystick && !m_joystickLockoutTranslate) {
        xSpeed = m_transXLockoutValue;
        ySpeed = m_transYLockoutValue;
      }
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
              : new ChassisSpeeds(xSpeed, ySpeed, rot));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(swerveModuleStates[0]);
      m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_rearLeft.setDesiredState(swerveModuleStates[2]);
      m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void forceStop() {
    m_frontLeft.forceStop();
    m_frontRight.forceStop();
    m_rearLeft.forceStop();
    m_rearRight.forceStop();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  /**
   * @brief Drives the robot to a distance following the current heading using a
   *        PID controller.
   * @param meters
   * @return the command
   */
  public Command driveXMetersPID(double meters) {
    return new FunctionalCommand(
        () -> InitMotionProfile(setpointToX(meters), setpointToY(meters)),
        () -> driveAuto(),
        (interrupted) -> {
          drive(0, 0, 0, true);
          m_joystickLockoutTranslate = false;
        },
        this::checkDone);
  }

  public Command centerOnTargetCommand(Boolean infinite) {
    return new FunctionalCommand(
        () -> {
          m_joystickLockoutRotate = true;
        },
        () -> centerOnTarget(),
        (interrupted) -> {
          drive(0, 0, 0, true);
          m_joystickLockoutRotate = false;
        },
        () -> !infinite && checkTurningDone());
  }


  public Command DriveToTargetCommand() {
    return DriveToTargetCommand(1.5, 1.0);

  }

  public Command DriveToTargetCommand(double speed, double maxDistance) {
    return new FunctionalCommand(
        () -> {
          m_joystickLockoutRotate = true;
          m_gotTarget = false;
          m_gotTargetCounter = 0;
          m_joystickLockoutTranslate = true;
          m_controller_theta.reset(m_gyro.getRotation2d().getRadians());
          m_startingPosition = getPose();
        },
        () -> driveToTarget(speed, maxDistance),
        (interrupted) -> {
          drive(0, 0, 0, true);
          m_joystickLockoutRotate = false;
          m_joystickLockoutTranslate = false;
        },
        () -> m_gotTarget);
  }

  /**
   * @brief Determines the x value of the setpoint based on the current heading
   * @param setpoint in meters
   * @return
   */
  private double setpointToX(double setpoint) {
    return setpoint * getPose().getRotation().getCos() + getPose().getX();
  }

  /**
   * @brief Determines the y value of the setpoint based on the current heading
   * @param setpoint in meters
   * @return
   */
  private double setpointToY(double setpoint) {
    return setpoint * getPose().getRotation().getSin() + getPose().getY();
  }

  /**
   * @brief Initialize the motion profile for the PID controller
   * @param setpointX
   * @param setpointY
   */
  private void InitMotionProfile(double setpointX, double setpointY) {

    m_controller_x.reset(getPose().getX());
    m_controller_x.setTolerance(.05);
    m_controller_x.setGoal(new TrapezoidProfile.State(setpointX, 0));

    m_controller_y.reset(getPose().getY());
    m_controller_y.setTolerance(.05);
    m_controller_y.setGoal(new TrapezoidProfile.State(setpointY, 0));

    m_joystickLockoutTranslate = true; // begin locking out the 0 values from the joystick

  }

  /**
   * @brief logic to drive the robot in auto-distance mode
   */
  private void driveAuto() {

    double pid_valX = m_controller_x.calculate(getPose().getX());
    double vel_pid_valX = pid_valX * DriveConstants.kMaxSpeedMetersPerSecond;

    double pid_valY = m_controller_y.calculate(getPose().getY());
    double vel_pid_valY = pid_valY * DriveConstants.kMaxSpeedMetersPerSecond;

    SmartDashboard.putNumber("Auto VelX", m_controller_x.getSetpoint().velocity + vel_pid_valX);
    SmartDashboard.putNumber("Auto VelY", m_controller_y.getSetpoint().velocity + vel_pid_valY);

    double finalVelX = m_controller_x.getSetpoint().velocity + vel_pid_valX;
    double finalVelY = m_controller_y.getSetpoint().velocity + vel_pid_valY;

    drive(finalVelX, finalVelY, 0, true, false);
  }

  private double degressToRadians(double degrees) {
    return degrees * Math.PI / 180;
  }

  private double calculateRotationToTarget() {
    double heading = m_gyro.getRotation2d().getRadians();

    double degPi = ySub.get();
    double finalVelTheta = -100;
    if (degPi != -1.0) {
      SmartDashboard.putNumber("degPi", degPi);
      SmartDashboard.putNumber("Heading", heading);
      SmartDashboard.putNumber("Dest", heading - degressToRadians(degPi));

      final double turnOutput = m_controller_theta.calculate(heading,
          heading - degressToRadians(degPi));

      SmartDashboard.putNumber("vel", m_controller_theta.getSetpoint().velocity);

      double vel_pid_theta = turnOutput * DriveConstants.kMaxSpeedMetersPerSecond;

      finalVelTheta = m_controller_theta.getSetpoint().velocity + vel_pid_theta;

      m_rotateLockoutValue = finalVelTheta;
    }
    return finalVelTheta;
  }

  private void centerOnTarget() {

    double rotationVel = calculateRotationToTarget();
    if (rotationVel > -100) {
      drive(0, 0, rotationVel, false, false);
    }

    SmartDashboard.putBoolean("At Target 1", checkTurningDone());
    SmartDashboard.putBoolean("At Target 2", m_controller_theta.atGoal());

  }

  private double getTotalDisplacement(){
    var pose = getPose();
    return Math.sqrt((pose.getX() - m_startingPosition.getX()) * (pose.getX() - m_startingPosition.getX()) + 
                     (pose.getY() - m_startingPosition.getY()) * (pose.getY() - m_startingPosition.getY()));
  }

  private double getTotalRotation(){
    return Math.abs(getPose().getRotation().getRadians() - m_startingPosition.getRotation().getRadians());
  }

  private void driveToTarget(double speed, double maxDistance) {
    double rotationVel = calculateRotationToTarget();
    if (rotationVel > -100) {
      drive(speed, 0, rotationVel, false, false);
    } else {
      m_gotTargetCounter++;

    }

    if (m_gotTargetCounter > 10) {
      m_gotTarget = true;
    }

    if((maxDistance > 0 && getTotalDisplacement() > maxDistance) || getTotalRotation() > Math.PI/4 ){
      //this is an error check to ensure we can't just run-away if we are fully auto here
      m_gotTarget = true;
    }

    SmartDashboard.putBoolean("m_gotTarget", m_gotTarget);
  }

  private boolean checkTurningDone() {
    return Math.abs(ySub.get()) < 2;
  }

  /**
   * @brief Checks if the robot is at the setpoint
   * @return
   */
  private boolean checkDone() {
    return m_controller_x.atGoal() && m_controller_y.atGoal();
  }

}
