package frc.robot.subsystems.escalator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Types.DirectionType;

//1 is escalator
//2 is lower intake
//8 is flipper 
//9 is elevator

public class EscalatorAssemblySubsystem extends SubsystemBase {

    private EscalatorSubsystem escalator;
    private ElevatorSubsystem elevator;
    private FlipperSubsystem flipper;

    public EscalatorAssemblySubsystem() {
        escalator = new EscalatorSubsystem(Constants.EscalatorConstants.pidValues,
                Constants.EscalatorConstants.ffValues, Constants.EscalatorConstants.deviceId, "Escalator");

        elevator = new ElevatorSubsystem(Constants.ElevatorContstraints.pidValues, Constants.ElevatorContstraints.limits,
         Constants.ElevatorContstraints.deviceId, "Elevator");

        flipper  = new FlipperSubsystem(Constants.FlipperContstraints.deviceId, Constants.FlipperContstraints.speed,"Flipper");
    }   

    @Override
    public void periodic() {

    }

    /*****************************************
     * Manual Control
     ******************************************/

    public Command RunEscalatorManualSpeedCommand(DoubleSupplier getSpeed) {
        return escalator.RunEscalatorManualSpeedCommand(getSpeed);
    }

    public Command RunElevatorManualSpeedCommand(DoubleSupplier getSpeed, Boolean ignoreLimits) {
        return elevator.RunElevatorManualSpeedCommand(getSpeed, ignoreLimits);
    }

    public Command RunFlipperManualSpeedCommand(DoubleSupplier getSpeed) {
        return flipper.RunFlipperManualSpeedCommand(getSpeed);
    }


    /*****************************************
     * Auto Control
     ******************************************/
    public Command RunFlipperToPositionCommand(DirectionType d){
        return flipper.RunFlipperToPositionCommand(d);
    }

    public Command RunEscalatorToPositionCommand(double position) {
        return escalator.RunEscalatorToPositionCommand(position);
    }

    public Command RunElevatorToPositionCommand(double position) {
        return elevator.RunElevatorToPositionCommand(position);
    }


    /*****************************************
     * Misc Control
     ******************************************/
    public Command ResetElevatorEncoderCommand(){
        return elevator.ResetElevatorEncoderCommand();
    }

}
