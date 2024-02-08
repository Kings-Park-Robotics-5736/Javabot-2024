package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.Types.PositionType;

public class ClimbSubsystem extends SubsystemBase {

    private ClimbMotorSubsystem right;
    private ClimbMotorSubsystem left;

    public ClimbSubsystem() {

        left = new ClimbMotorSubsystem(Constants.LeftClimbConstants.kPidValues,
                Constants.LeftClimbConstants.kFFValues, Constants.LeftClimbConstants.kDeviceId, "Left",true);

         right= new ClimbMotorSubsystem(Constants.RightClimbConstants.kPidValues,
                Constants.RightClimbConstants.kFFValues, Constants.RightClimbConstants.kDeviceId, "Right",false);

    }

    @Override
    public void periodic() {

    }


    public Command RunClimbForwardCommand() {
        return Commands.parallel(right.RunClimbForwardCommand(), left.RunClimbForwardCommand());
    }

    public Command RunClimbBackwardCommand() {
        return Commands.parallel(right.RunClimbBackwardCommand(), left.RunClimbBackwardCommand());
    }





    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, PositionType whichClimb) {
        if (whichClimb == PositionType.LEFT) {
            return right.sysIdQuasistatic(direction);
        } else {
            return left.sysIdQuasistatic(direction);
        }

    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction, PositionType whichClimb) {
         if (whichClimb == PositionType.RIGHT) {
            return right.sysIdDynamic(direction);
        } else {
            return left.sysIdDynamic(direction);
        } 
    }

}
