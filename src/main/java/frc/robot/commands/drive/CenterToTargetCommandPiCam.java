package frc.robot.commands.drive;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.vision.PiCamera;

/**
 * @brief This command centers the robot on the target. It does NOT drive to it
 *        It is useful for a user to manually hold down a button, for instance,
 *        and manually drive to an object
 * 
 * @note Because we want the user to still be able to translate robot, we will
 *       not require robotDrive as a subsystem. Otherwise all joystick controls
 *       will be locked out. As such, we will manually lock out the rotate joystick.
 */
public class CenterToTargetCommandPiCam extends CenterToTargetCommand {

    PiCamera m_picam;
    private final double PICAM_ERROR_THRESH = 2;

    public CenterToTargetCommandPiCam(DriveSubsystem robot_drive, PiCamera picam, boolean infinite) {

        super(robot_drive, infinite);
        this.m_picam = picam;

        // NOTE - we explicitly do not do the below line, or else we can't drive while
        // centering
        // addRequirements(m_drive);

    }

    @Override
    public void execute() {
        centerOnTarget(m_picam.getPiCamAngle(), true);
    }

    protected boolean checkTurningDone() {
        return Math.abs(m_picam.getPiCamAngle()) < PICAM_ERROR_THRESH;
    }

}
