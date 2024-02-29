package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.field.ScoringPositions;

public class MathUtils {
    public static double distanceBetweenPoses(Pose2d pose_a, Pose2d pose_b){
        var xDelta = pose_a.getTranslation().getX() - pose_b.getTranslation().getX();
        var yDelta = pose_a.getTranslation().getY() - pose_b.getTranslation().getY();
        var angleToTarget = Math.atan(yDelta / xDelta);  // normally tan is x/y, but in frc coords, it
                                                                         // is y / x
                                                       
        return angleToTarget;                                                                 
    }

    public static double distanceToScoringTarget(Pose2d robotPose){
        var alliance = DriverStation.getAlliance();
        Pose2d scoringPos;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            scoringPos = ScoringPositions.kBlueScoringPosition;
        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            scoringPos = ScoringPositions.kRedScoringPosition;
        } else {
            return 0;
        }

        return distanceBetweenPoses(robotPose, scoringPos);
    }
}
