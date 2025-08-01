package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorMoveComand extends Command{
    private final Elevator elevator;
    private final double elevatorTargetPosition;

    public ElevatorMoveComand(Elevator elevator, double elevatorTargetPosition){
        this.elevatorTargetPosition = elevatorTargetPosition;

        this.elevator = elevator;

        addRequirements(elevator);

    }

    @Override
    public void initialize(){
        elevator.SetPos(elevatorTargetPosition);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}