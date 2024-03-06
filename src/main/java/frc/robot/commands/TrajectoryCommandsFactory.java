package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class TrajectoryCommandsFactory {

    /**
     * @brief calculate a trajectory given a start and an end
     * @param start
     * @param end
     * @return
     */

    public static Command generatePPTrajectoryCommand(DriveSubsystem robotDrive, Pose2d endPos, Rotation2d endRotation) {
        return Commands.runOnce(() -> {
            Pose2d currentPose = robotDrive.getPose();

            // The rotation component in these poses represents the direction of travel
            Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
           
            List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
            PathPlannerPath path = new PathPlannerPath(
                    bezierPoints,
                    new PathConstraints(
                            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                            Units.degreesToRadians(360), Units.degreesToRadians(540)),
                    new GoalEndState(0.0, endRotation));
            path.preventFlipping = true;

            AutoBuilder.followPath(path).schedule();
        });
    }

    public static Command getPathFollowCommandAmp() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Amp Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }

        public static Command getPathFollowCommandTrap() {
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Trap Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }


}
