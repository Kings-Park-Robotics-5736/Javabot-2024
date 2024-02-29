package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.CenterToGoalCommand;
import frc.robot.field.ScoringPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcherAssembly.LauncherAssemblySubsystem;
import frc.robot.utils.MathUtils;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiCamera;

public class RobotCommandsFactory {



    public static Command RunFloorInatakeForwardWithShooterIntakeCommand(IntakeSubsystem intake, LauncherAssemblySubsystem launcher) {        
            return new ParallelCommandGroup(intake.RunIntakeForwardCommand(), launcher.RunShooterBackwardCommand());//.until(()->launcher.ArmContainsNote());
    }

    public static Command RunFloorIntakeWithArmPosition(IntakeSubsystem intake, LauncherAssemblySubsystem launcher){
        return launcher.RunArmToIntakePositionCommand().andThen(RunFloorInatakeForwardWithShooterIntakeCommand(intake, launcher));
    }

    /**
     * Automatically shoot once we get in the range of the target, with target centering active
     * @param launcher
     * @param drive
     * @return
     */
    public static Command ShootWhenInRange(LauncherAssemblySubsystem launcher, DriveSubsystem drive){
        return launcher.RunShooterForwardCommand().alongWith(new CenterToGoalCommand(drive, true)).until(()->MathUtils.distanceToScoringTarget(drive.getPose()) < ScoringPositions.maxDistanceToScoreMeters).andThen(launcher.RunShooterAndKickupForwardCommand());
    }



    /**
     * @brief Generate a command that will start where it is, drive to a target
     *        using PiCam vision, intake target,
     *        drive to a scoring position, and score the target.
     * @param robotDrive    subsystem
     * @param picam         pi camera
     * @param limelight     limelight camera
     * @param intake        intake subsystem
     * @param escalator     escalator subsystem
     * @param finalPosition final position to drive to
     * @param height        height to score at
     * @return command
     */

     //TODO
    public static Command generateGrabTargetAndScoreCommmand(DriveSubsystem robotDrive, PiCamera picam,
            Limelight limelight,
            Pose2d finalPosition) {

        return null;
    }


    /**
     * @brief Generate a command that combines the drive to target with a running intake command.
     * @note Command will terminate when the drive to target command terminates, since intake is infinite
     * @param robotDrive
     * @param picam
     * @param intake
     * @return
     */
    public static Command generateDriveToTargetWithIntake(DriveSubsystem robotDrive, PiCamera picam,
            double speed, double maxDistance) {

        return null;
    }


    /**
     * @brief Generate a command that will use path planning to drive to pose, with 2 path planning iterations
     *        to ensure we get to the final spot we want.
     * @param robotDrive
     * @param finalPosition
     * @return
     */
    public static Command generateDoublePathPlanningCommand(DriveSubsystem robotDrive, Pose2d finalPosition, Rotation2d finalRotation) {

        return TrajectoryCommandsFactory.generatePPTrajectoryCommand(robotDrive,
                finalPosition, finalRotation).andThen(
                        TrajectoryCommandsFactory.generatePPTrajectoryCommand(robotDrive, finalPosition, finalRotation));
    }


    /**
     * @brief Generate a command that will use path planning to drive to pose, with 2 path planning iterations
     *       to ensure we get to the final spot we want. This command will also set the limelight to the
     *       reflective pipeline before starting the 2nd path planning, since switching pipelines takes time.
     * @param robotDrive
     * @param limelight
     * @param finalPosition
     * @return
     */
    public static Command generateDoublePathPlanningCommandWithLimelightTargeting(DriveSubsystem robotDrive,
            Limelight limelight, Pose2d finalPosition) {

        return null;
    }

  

    /**
     * Generate the command to score to an object a specific position (height), while centering with limelight.
     * @param robotDrive
     * @param escalator
     * @param limelight
     * @param height
     * @return
     */
    public static Command generateScoreCommandWithCentering(DriveSubsystem robotDrive,
            Limelight limelight) {

        return null;
    }

}
