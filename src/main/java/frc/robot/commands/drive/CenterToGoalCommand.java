package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.Constants.CenterToFieldPositionConstants;
import frc.robot.field.ScoringPositions;

/**
 * @brief This command centers the robot to a specific point on the field. It
 *        does NOT drive to it
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls
 *       will be locked out. As such, we will manually lock out the rotate
 *       joystick.
 */
public class CenterToGoalCommand extends CenterToTargetCommand {

    private final double POSE_ERROR_THRESH = 2;
    private double m_goal_rotation = 0;


    public CenterToGoalCommand(DriveSubsystem robot_drive, boolean infinite) {

        // call the parent constructor.
        super(robot_drive, infinite, new TrapezoidProfile.Constraints(
            CenterToFieldPositionConstants.kMaxSpeedMetersPerSecond, CenterToFieldPositionConstants.kMaxAccelerationMetersPerSecondSquared), CenterToFieldPositionConstants.kPidValues); 

    }

    @Override
    public void initialize() {
        m_drive.setJoystickRotateLockout(true, true);
        m_controller_theta.reset(m_drive.getPose().getRotation().getRadians());
        m_controller_theta.setTolerance(0.01);
        m_controller_theta.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    protected void centerOnTarget(double angle, boolean useCameraMeasurement) {
        double rotationVel = DriveCommandsCommon.calculateRotationToFieldPos(
                m_drive.getPose().getRotation().getRadians(),
                useCameraMeasurement,
                angle, m_controller_theta);
        m_drive.setRotateLockoutValue(rotationVel);
        m_drive.drive(0, 0, rotationVel, true, false);
    }

    @Override
    public void execute() {

        // calculate the angle of our current pose to the target
        Pose2d robotPose = m_drive.getPose();
        Pose2d scoringPos;
        double rotationOffset = 0;

        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            scoringPos = ScoringPositions.kBlueScoringPosition;
            rotationOffset += Units.degreesToRadians(180); //when facing the blue side of the field, that is 180 deg.
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            scoringPos = ScoringPositions.kRedScoringPosition;
        } else {
            return;
        }

        var xDelta = robotPose.getTranslation().getX() - scoringPos.getTranslation().getX();
        var yDelta = robotPose.getTranslation().getY() - scoringPos.getTranslation().getY();
        var angleToTarget = Math.atan(yDelta / xDelta) + rotationOffset; // normally tan is x/y, but in frc coords, it
                                                                         // is y / x
        m_goal_rotation = angleToTarget;
        centerOnTarget(angleToTarget, true);

    }

    protected boolean checkTurningDone() {
        return Math.abs(m_goal_rotation - m_drive.getPose().getRotation().getRadians() ) < POSE_ERROR_THRESH;
    }

}