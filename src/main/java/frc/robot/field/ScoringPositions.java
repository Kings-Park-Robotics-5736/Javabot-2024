package frc.robot.field;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.MathUtils;

public final class ScoringPositions {
    public static final Point2D kChargeStationBlue3 = new Point2D.Double(1.96, 3.87);
    public static final Pose2d kRobotPoseChargeStationBlue3 = new Pose2d(new Translation2d(kChargeStationBlue3.getX(), kChargeStationBlue3.getY()), new Rotation2d(MathUtils.degreesToRadians(180)));
}
