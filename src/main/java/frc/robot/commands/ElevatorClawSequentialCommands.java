package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ElevatorClawSequentialCommands extends SequentialCommandGroup{
    public ElevatorClawSequentialCommands(Elevator elevator, Claw claw, double elevatorTargetPosition, double ClawTargetPosition){
        addCommands(
            new ElevatorMoveComand(elevator, elevatorTargetPosition),
            new WaitCommand(0.35),
            new ClawMoveCommand(claw, ClawTargetPosition)
        );
    }
}
