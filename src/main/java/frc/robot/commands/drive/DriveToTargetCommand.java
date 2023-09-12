package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.PiCamera;

/**
 * @brief This command is like CenterToTarget, except it will also drive at a
 *        static speed until it reaches the target
 * 
 * @not Reaching target is defined as no longer being able to see it.
 */
public class DriveToTargetCommand extends CommandBase {
    private DriveSubsystem m_drive;
    private boolean m_gotTarget;
    private int m_gotTargetCounter;
    private Pose2d m_startingPosition;
    private boolean m_inSlowdown;
    private int m_slowdownCounter;
    private double m_slowdownSpeed;

    private final PiCamera m_picam;
    private final double m_speed;
    private final double m_maxDistance;
    private double m_useSpeed;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            DriveConstants.kMaxSpeedMetersPerSecond, 2);

    private final ProfiledPIDController m_controller_theta = new ProfiledPIDController(1, 0, 0.000, m_constraints,
            Constants.kDt);

    public DriveToTargetCommand(DriveSubsystem robot_drive, PiCamera picam, double speed, double maxDistance) {
        this.m_drive = robot_drive;

        this.m_picam = picam;
        this.m_speed = speed;
        this.m_maxDistance = maxDistance;

        addRequirements(m_drive);

    }

    @Override
    public void initialize() {
        m_drive.setJoystickRotateLockout(true);
        m_gotTarget = false;
        m_gotTargetCounter = 0;
        m_drive.setJoystickTranslateLockout(true);
        m_controller_theta.reset(m_drive.getHeadingInRadians());
        m_startingPosition = m_drive.getPose();

        // SmartDashboard.putBoolean("DriveToTarget", true);

        // m_controller_theta.enableContinuousInput(-Math.PI, Math.PI);
        m_controller_theta.setTolerance(0.01);

        m_inSlowdown = false;
        m_slowdownCounter = 0;
        m_useSpeed = m_speed;//0.5;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);

        m_drive.setJoystickRotateLockout(false);
        m_drive.setJoystickTranslateLockout(false);
        // SmartDashboard.putBoolean("DriveToTarget", false);
    }

    @Override
    public void execute() {
        if (m_inSlowdown) {
            slowDown();
        } else {
            driveToTarget(m_speed, m_maxDistance);
        }
    }

    @Override
    public boolean isFinished() {
        return m_gotTarget && !m_inSlowdown;
    }

    private double getTotalDisplacement() {
        var pose = m_drive.getPose();
        return Math.sqrt((pose.getX() - m_startingPosition.getX()) * (pose.getX() - m_startingPosition.getX()) +
                (pose.getY() - m_startingPosition.getY()) * (pose.getY() - m_startingPosition.getY()));
    }

    private double getTotalRotation() {
        return Math.abs(m_drive.getPose().getRotation().getRadians() - m_startingPosition.getRotation().getRadians());
    }

    private void slowDown() {
        /*
         * if(m_slowdownCounter < 200){
         * m_slowdownCounter++;
         * }else{
         * m_inSlowdown=false;
         * }
         * 
         * m_drive.drive(m_slowdownSpeed, 0, 0, false, false);
         * if(m_slowdownCounter%10==0){
         * m_slowdownSpeed/=2.0;
         * }
         * System.out.println("Setting drive speed to " + m_slowdownSpeed);
         */
        m_inSlowdown = false;

    }

    private void driveToTarget(double speed, double maxDistance) {
        var piAngle = m_picam.getPiCamAngle();
        if (Math.abs(piAngle) < 50) {
            double rotationVel = DriveCommandsCommon.calculateRotationToTarget(m_drive.getHeadingInRadians(), true,
                    piAngle,
                    m_controller_theta);
            if (rotationVel > -100) {

                m_drive.setRotateLockoutValue(rotationVel);
                m_drive.drive(m_useSpeed, 0, rotationVel, false, false);
                m_gotTargetCounter = 0;
            } else {
                m_gotTargetCounter++;

            }
        } else {
            m_gotTargetCounter++;
        }
        if (m_useSpeed < m_speed) {
            m_useSpeed += .08;
        } else {
            m_useSpeed = m_speed;
        }

        if (m_gotTargetCounter > 30) {
            m_gotTarget = true;
            m_inSlowdown = true;
            m_slowdownSpeed = m_speed;
            System.out.println("---------------Los Target-----------------");
        }

        if ((maxDistance > 0 && getTotalDisplacement() > maxDistance) || getTotalRotation() > Math.PI / 4) {
            // this is an error check to ensure we can't just run-away if we are fully auto
            // here
            System.out.println("---------------Overran-----------------");
            m_gotTarget = true;
            m_inSlowdown = true;
            m_slowdownSpeed = m_speed;
        }

        // SmartDashboard.putBoolean("m_gotTarget", m_gotTarget);
    }

}