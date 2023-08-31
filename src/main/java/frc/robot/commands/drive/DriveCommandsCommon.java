package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.MathUtils;

public class DriveCommandsCommon {
    public static  double calculateRotationToTarget(double heading, double degPi, ProfiledPIDController m_controller_theta) {
       
        double finalVelTheta = -100;
        if (degPi != -1.0) {
          SmartDashboard.putNumber("degPi", degPi);
          SmartDashboard.putNumber("Heading", heading);
          SmartDashboard.putNumber("Dest", heading - MathUtils.degreesToRadians(degPi));
    
          final double turnOutput = m_controller_theta.calculate(heading,
              heading - MathUtils.degreesToRadians(degPi));
    
          SmartDashboard.putNumber("vel", m_controller_theta.getSetpoint().velocity);
    
          double vel_pid_theta = turnOutput * DriveConstants.kMaxSpeedMetersPerSecond;
    
          finalVelTheta = m_controller_theta.getSetpoint().velocity + vel_pid_theta;
    
          
        }
        return finalVelTheta;
      }
}
