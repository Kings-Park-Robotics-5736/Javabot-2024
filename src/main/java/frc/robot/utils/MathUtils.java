package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.field.ScoringPositions;

public class MathUtils {


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

        return scoringPos.getTranslation().getDistance(robotPose.getTranslation());
    }

    public static double angleRadiansToScoringTarget(Pose2d robotPose){
        //0.4064 is robot rotation height
        double distance = distanceToScoringTarget(robotPose);
        System.out.println("Distance to Scoring Target " + distance );
        double newAngle = - Math.toRadians(3.41 * distance * distance -26.5* distance + 76);
        double modelAngle =  - Math.atan((ScoringPositions.speakerOpeningFromFloorMeters - 0.438)/(distance));
        System.out.println( " New angle "  + Math.toDegrees(newAngle) + ", Old Angle: " + Math.toDegrees(modelAngle));
        return newAngle - Math.toRadians(2);

        //return - Math.atan((ScoringPositions.speakerOpeningFromFloorMeters - 0.438)/(distance)) + Math.toRadians(-1.6* distance + 5.9);
    }
}
