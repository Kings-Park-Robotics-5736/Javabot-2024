// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.
 * This class should not be used for any other purpose. All constants
 * should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kDt = 0.02;

  public static final class DriveConstants {

    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 4;
    public static final int kFrontRightDriveMotorPort = 0;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 7;

    public static final int kFrontLeftTurningEncoderPort = 1;
    public static final int kRearLeftTurningEncoderPort = 2;
    public static final int kFrontRightTurningEncoderPort = 0;
    public static final int kRearRightTurningEncoderPort = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveReversed = false;
    public static final boolean kRearLeftDriveReversed = false;
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kRearRightDriveReversed = true;

    public static final double kFrontLeftAngleOffset = -94.395;
    public static final double kFrontRightAngleOffset = -68.8;
    public static final double kBackLeftAngleOffset = -257.52;
    public static final double kBackRightAngleOffset = -316.143;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.482;

    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.61;

    public static final String kCanName = "Canivore";

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double ksVoltsTurning = .10059;
    public static final double kvVoltSecondsPerMeterTurning = 0.3749;
    public static final double kaVoltSecondsSquaredPerMeterTurning = 0.1142;
    public static final double ksVoltsDrive = .26912;// .22125;
    public static final double kvVoltsDrive = 2.1702;// 2.1898;
    public static final double kaVoltsDrive = 0.509;// 0.58667;
    public static final double kPDrive = 2.5049;// 1.8044;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 15 * 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 15 * 2 * Math.PI;

    public static final int kEncoderResolution = 2048;
    public static final double kWheelRadius = 0.0508;
    public static final double kDriveGearRatio = 6.75;

    public static final double kPModuleTurningController = 7.9245;
    public static final double kPModuleTurningNoController = 5;

  }

  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kActionControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 5;
    public static final double kPYController = 5;
    public static final double kPThetaController = 5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class IntakeConstants {

    public static final int kForwardSpeed = 4000;
    public static final int kReverseSpeed = -3000;
  }

  public static final class LowerIntakeConstants {

    public static final byte kDeviceId = 2;

    public static final PidConstants kPidValues = new PidConstants(0.00001, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.06368, 0.12005, 0.0034381);
  }

  public static final class UpperIntakeConstants {

    public static final byte kDeviceId = 3;

    public static final PidConstants kPidValues = new PidConstants(0.00001, 0, 0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.06368, 0.12005, 0.0034381);
  }

  public static final class EscalatorConstants {

    public static final byte kDeviceId = 1;

    public static final PidConstants kPidValues = new PidConstants(0.06, .0001, 0.0);
    public static final FeedForwardConstants kFFValues = new FeedForwardConstants(0.27419, 0.12313, 0.0026, 0.11545);
    public static final Limits kLimits = new Limits(0, 62);

    public static final int kMaxVelocity = 120;
    public static final int kMaxAcceleration = 150;

    public static final double kStaleTolerance = .75;
    public static final double kDiffThreshold = 0.25;
    public static final int kStaleThreshold = 5;
  }

  public static final class ElevatorConstants {

    public static final byte kDeviceId = 9;

    public static final PidConstants kPidValues = new PidConstants(0.5, 0, 0.007);
    public static final Limits kLimits = new Limits(0, 60);

    public static final int kMaxVelocity = 100;
    public static final int kMaxAcceleration = 100;

    public static final double kStaleTolerance = .5;
    public static final double kDiffThreshold = 0.15;
    public static final int kStaleThreshold = 10;
    public static final double kPositionTolerance = 1.0;
  }

  public static final class FlipperContstants {

    public static final byte kDeviceId = 8;

    public static final double kSpeed = .75;

    public static final double kDownTime = 0.6;
    public static final double kUpTime = 0.8;
  }

  public static final class DriveToTargetCommandConstants {

    // after 30 times of not seeing target after seeing it, declare it intaked
    public static final int kGotTargetThreshold = 30;

    public static final double kMaxRotation = Math.PI / 4; // 45 degrees
    public static final PidConstants kPidValues = new PidConstants(1, 0, 0.00);

  }

  public static final class CustomDriveDistanceCommandConstants {
    public static final PidConstants kPidValues = new PidConstants(.5, .1, 0.00);

  }

}
