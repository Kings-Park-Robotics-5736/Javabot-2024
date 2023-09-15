package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.CenterToTargetCommandLimelight;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.drive.PathPlanFromDynamicStartCommand;
import frc.robot.field.ScoringPositions;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.escalator.EscalatorAssemblySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.utils.Types;
import frc.robot.utils.Types.ScoringHeight;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiCamera;

public class RobotCommandsFactory {

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
    public static Command generateGrabTargetAndScoreCommmand(DriveSubsystem robotDrive, PiCamera picam,
            Limelight limelight, IntakeSubsystem intake, EscalatorAssemblySubsystem escalator,
            Pose2d finalPosition, Types.ScoringHeight height) {

        return escalator.RunElevatorDownCommand()
                .andThen(generateDriveToTargetWithIntake(robotDrive, picam, intake,2,3))
                .andThen(escalator.RunElevatorUpCommand()
                        .alongWith(generateDoublePathPlanningCommandWithLimelightTargeting(robotDrive, limelight,
                                finalPosition)))
                .andThen(generateScoreCommandWithCentering(robotDrive, escalator, limelight, height));
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
            IntakeSubsystem intake, double speed, double maxDistance) {

        return intake.RunIntakeForwardCommand().raceWith(new DriveToTargetCommand(robotDrive, picam, speed, maxDistance));
    }


    /**
     * @brief Generate a command that will use path planning to drive to pose, with 2 path planning iterations
     *        to ensure we get to the final spot we want.
     * @param robotDrive
     * @param finalPosition
     * @return
     */
    public static Command generateDoublePathPlanningCommand(DriveSubsystem robotDrive, Pose2d finalPosition) {

        return new PathPlanFromDynamicStartCommand(robotDrive::getPose, robotDrive,
                finalPosition).andThen(
                        new PathPlanFromDynamicStartCommand(robotDrive::getPose, robotDrive,
                                finalPosition));
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

        return new PathPlanFromDynamicStartCommand(robotDrive::getPose, robotDrive,
                finalPosition, true, true).andThen(Commands.runOnce(() -> limelight.setReflectivePipeline()))
                .andThen(new PathPlanFromDynamicStartCommand(robotDrive::getPose, robotDrive,
                        finalPosition, true, true));
    }

    /**
     * Generate the entire score command to a specific scoring position. Uses generateGrabTargetAndScoreCommmand().
     * @param robotDrive
     * @param picam
     * @param limelight
     * @param intake
     * @param escalator
     * @return
     */
    public static Command generateScoreMidToBlueChargeStation3Command(DriveSubsystem robotDrive, PiCamera picam,
            Limelight limelight, IntakeSubsystem intake, EscalatorAssemblySubsystem escalator) {

        return generateGrabTargetAndScoreCommmand(robotDrive, picam, limelight, intake, escalator,
                ScoringPositions.kRobotPoseChargeStationBlue3, ScoringHeight.MID);
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
            EscalatorAssemblySubsystem escalator, Limelight limelight, Types.ScoringHeight height) {

        return getElevatorCommandFromHeight(escalator, height)
                .raceWith(new CenterToTargetCommandLimelight(robotDrive, escalator, limelight, true));
    }

    /**
     * Convert enum Scoring Height to a command that scores at that height.
     * @note Score Low is not currently supported.
     * @param escalator
     * @param height
     * @return
     */
    private static Command getElevatorCommandFromHeight(EscalatorAssemblySubsystem escalator,
            Types.ScoringHeight height) {

        Command ret;
        switch (height) {
            case MID:
                ret = escalator.ScoreMid();
                break;
            case HIGH:
                ret = escalator.ScoreHigh();
            default:
                ret = escalator.ScoreMid();
        }
        return ret;
    }
}
