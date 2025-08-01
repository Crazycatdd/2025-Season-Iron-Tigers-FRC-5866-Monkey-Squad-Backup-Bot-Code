package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class ClawElevatorSequentialCommands extends SequentialCommandGroup{
    public ClawElevatorSequentialCommands(Claw claw, Elevator elevator, double ClawTargetPosition, double elevatorTargetPosition){
        addCommands(
            new ClawMoveCommand(claw, ClawTargetPosition),
            new WaitCommand(1),
            new ElevatorMoveComand(elevator, elevatorTargetPosition)
        );
    }
}
