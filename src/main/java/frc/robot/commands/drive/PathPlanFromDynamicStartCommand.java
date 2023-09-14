package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TrajectoryCommandsFactory;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * @brief this is a wrapper around the path planning lib to allow us to go to a
 *        target based on our current starting position
 * 
 * @note Usual pplib requires giving the trajectory on command creation ('new'
 *       operator). This wrapper allows us to delay that until we call
 *       initialize().
 */
public class PathPlanFromDynamicStartCommand extends CommandBase {

    private PPSwerveControllerCommand m_ppscc;
    private final Supplier<Pose2d> m_initialPoseSupplier;
    private final DriveSubsystem m_robotDrive;
    private final boolean m_usePid;
    private final boolean m_continueTillAtPos;

    private Pose2d m_endPose;
    private ArrayList<PathPoint> m_mid_poses;
    private int m_pathPlannerDoneCounter;

    /**
     * @param initialPoseSupplier method to obtain the current robot pose to
     *                            determine initial state.
     * @param robotDrive
     * @param endPose             the final pose to drive to
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose) {
        this(initialPoseSupplier, robotDrive, endPose, new ArrayList<PathPoint>(), true, false);

    }

    /**
     * @brief This constructor allows you to specify a list of intermediate points
     * @param initialPoseSupplier method to obtain the current robot pose to
     * @param robotDrive
     * @param endPose             the final pose to drive to
     * @param midPoses            a list of intermediate points to drive to along
     *                            the way
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose, ArrayList<PathPoint> midPoses) {
        this(initialPoseSupplier, robotDrive, endPose, midPoses, true, false);

    }

    /**
     * @param initialPoseSupplier
     * @param robotDrive
     * @param endPose
     * @param usePid              whether to use PID or not. Helpful in auto if we
     *                            dont care about accuracy
     * @param continueTillAtPos   whether to continue till we are at the final
     *                            position, or stop when pathplanner says time is up
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose, boolean usePid, boolean continueTillAtPos) {
        this(initialPoseSupplier, robotDrive, endPose, new ArrayList<PathPoint>(), usePid, continueTillAtPos);

    }

    /**
     * @brief fully loaded constructor
     * @param initialPoseSupplier
     * @param robotDrive
     * @param endPose
     * @param midPoses
     * @param usePid
     * @param continueTillAtPos
     */
    public PathPlanFromDynamicStartCommand(Supplier<Pose2d> initialPoseSupplier, DriveSubsystem robotDrive,
            Pose2d endPose, ArrayList<PathPoint> midPoses, boolean usePid, boolean continueTillAtPos) {
        m_initialPoseSupplier = initialPoseSupplier;
        m_robotDrive = robotDrive;
        m_endPose = endPose;
        m_usePid = usePid;
        m_continueTillAtPos = continueTillAtPos;
        m_mid_poses = midPoses;
        addRequirements(robotDrive); // this effectively locks out the joystick controlls.
        // without this, the 0,0 from the joystick still happens, causing jittery
        // movement
    }

    /**
     * @brief allows you to set the end pose after the command has been created
     * @note Must be set before initialize() is called (command is started) for it to take effect
     * @param endPose
     */
    public void SetEndPose(Pose2d endPose) {
        m_endPose = endPose;
    }

    @Override
    public void initialize() {

        // delay trajectory creation until this initialize.
        var traj = TrajectoryCommandsFactory.generateTrajectory(m_initialPoseSupplier.get(), m_mid_poses, m_endPose);
        m_ppscc = TrajectoryCommandsFactory.generatePPTrajectoryCommand(m_robotDrive, traj, m_usePid);
        m_ppscc.initialize();
        m_pathPlannerDoneCounter = 0;
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("Counter = " + m_pathPlannerDoneCounter + ", poseX = " + m_robotDrive.getPose().getX()
        //        + ", poseY = " + m_robotDrive.getPose().getY() + ", desired X = " + m_endPose.getX() + ", desiredY = "
        //        + m_endPose.getY());
        m_ppscc.end(interrupted);
    }

    @Override
    public void execute() {
        m_ppscc.execute();
    }

    @Override
    public boolean isFinished() {

        if (m_continueTillAtPos) {
            if (m_ppscc.isFinished()) {
                m_pathPlannerDoneCounter++;
            }
            return m_pathPlannerDoneCounter > 25 || (Math.abs(m_robotDrive.getPose().getX() - m_endPose.getX()) < .02
                    && Math.abs(m_robotDrive.getPose().getY() - m_endPose.getY()) < .02);
        } else {
            return m_ppscc.isFinished();
        }
    }

}
