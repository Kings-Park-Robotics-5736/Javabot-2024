package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.HuntAndReturnCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.PiCamera;

public class TrajectoryCommandsFactory {

 

    /**
     * @brief Generates a command that will follow an autonomous path group with
     *        events
     * @param robotDrive subsystem
     * @param intake     subsystem
     * @param escalator  subsystem
     * @return
     */
    public static Command generateAutoTrajectoryCommand(DriveSubsystem robotDrive, CommandMap commandMap) {

        // Load the path group from the path planner. This is a list of trajectories
        // that will be run in sequence.
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("mainWithDriveOnly", new PathConstraints(1, 1));

        if (pathGroup == null) {
            return new PrintCommand("Path group is null. Ensure path file is on the rio");
        }

        var eventMap = commandMap.getRawMap();
      
        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                robotDrive::getPose, // Pose2d supplier
                robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                new PIDConstants(AutoConstants.kPXController, 0.0, 0.0),
                new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0),
                robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                robotDrive // The drive subsystem. Used to properly set the requirements of path following
                           // commands
        );

        return autoBuilder.fullAuto(pathGroup);
    }

    /**
     * @brief calculate a trajectory given a start and an end
     * @param start
     * @param end
     * @return
     */
    public static PathPlannerTrajectory generateTrajectory(Pose2d start, Pose2d end) {
        return generateTrajectory(start, new ArrayList<PathPoint>(), end);
    }

    /**
     * @brief Calculate a trajectory given a start, mid points, and an end.
     * @param start
     * @param points
     * @param end
     * @return
     */
    public static PathPlannerTrajectory generateTrajectory(Pose2d start, List<PathPoint> points, Pose2d end) {
        PathPlannerTrajectory traj1;
        if (points.size() > 0) {
            points.add(new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()));
            traj1 = PathPlanner.generatePath(
                    new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                    new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()), // position,
                                                                                                     // heading
                    points.get(0),
                    points.subList(1, points.size()).toArray(new PathPoint[points.size()-1]));
        } else {
            traj1 = PathPlanner.generatePath(
                    new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                    new PathPoint(start.getTranslation(), start.getRotation(), start.getRotation()), // position,
                                                                                                     // heading
                    new PathPoint(end.getTranslation(), end.getRotation(), end.getRotation()) // position, heading
            );
        }
        return traj1;
    }

    /**
     * @brief Generates a command that will follow a path, with a start and end
     *        point
     * @param robotDrive subsystem
     * @param start      start point
     * @param end        end point
     * @return
     */
    public static Command generateTrajectoryCommand(DriveSubsystem robotDrive, Pose2d start, Pose2d end) {
        return generateTrajectoryCommand(robotDrive, start, end, false);
    }

    /**
     * @brief Generates a command that will follow a path, with a start and end
     *        point, and an option to reset odometry if first path
     * @param robotDrive
     * @param start
     * @param end
     * @param isFirstPath whether or not this is the first time and to reset
     *                    odometry
     * @return
     */
    public static Command generateTrajectoryCommand(DriveSubsystem robotDrive, Pose2d start, Pose2d end,
            Boolean isFirstPath) {
        return generateTrajectoryCommand(robotDrive, start, new ArrayList<PathPoint>(), end, isFirstPath);

    }

    /**
     * @brief Generates a command that will follow a path, with a start and end
     *        point, and intermediate points
     * @param robotDrive
     * @param start
     * @param points      points in between start and end. These are used to
     *                    generate a smooth path
     * @param end
     * @param isFirstPath
     * @return
     */
    public static Command generateTrajectoryCommand(DriveSubsystem robotDrive, Pose2d start, List<PathPoint> points,
            Pose2d end, Boolean isFirstPath) {

        var traj = generateTrajectory(start, points, end);

        return generateTrajectoryCommand(robotDrive, traj, isFirstPath);

    }

    /**
     * @brief Generates a command that will follow the given path.
     * @param robotDrive
     * @param traj1       the trajectory path to follow
     * @param isFirstPath
     * @return
     */
    public static Command generateTrajectoryCommand(DriveSubsystem robotDrive, PathPlannerTrajectory traj1,
            Boolean isFirstPath) {

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        robotDrive.resetOdometry(traj1.getInitialHolonomicPose());
                    }
                }),
                generatePPTrajectoryCommand(robotDrive, traj1),
                new InstantCommand(() -> {
                    robotDrive.drive(0, 0, 0, false);
                })

        );
    }

    public static PPSwerveControllerCommand generatePPTrajectoryCommand(DriveSubsystem robotDrive,
            PathPlannerTrajectory traj1) {
        return generatePPTrajectoryCommand(robotDrive, traj1, true);
    }

    /**
     * @brief Generates a command that will follow the given path.
     * @param robotDrive
     * @param traj1       the trajectory path to follow
     * @param isFirstPath
     * @return
     */
    public static PPSwerveControllerCommand generatePPTrajectoryCommand(DriveSubsystem robotDrive,
            PathPlannerTrajectory traj1, boolean usePid) {

        return new PPSwerveControllerCommand(
                traj1,
                robotDrive::getPose, // Pose supplier
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics

                // X controller.
                new PIDController(usePid ? AutoConstants.kPXController : 0, 0, 0),

                // Y controller (usually the same valuesas X controller)
                new PIDController(usePid ? AutoConstants.kPYController : 0, 0, 0),
                new PIDController(usePid ? AutoConstants.kPThetaController : 0, 0, 0), // Rotation controller.
                robotDrive::setModuleStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color.
                      // Optional, defaults to true
                robotDrive // Requires this drive subsystem
        );

    }

}
