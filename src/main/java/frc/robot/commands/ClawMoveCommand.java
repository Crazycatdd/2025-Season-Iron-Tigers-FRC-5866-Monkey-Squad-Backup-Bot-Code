package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class ClawMoveCommand extends Command{
    Claw claw;
    double ClawTargetPosition;

    public ClawMoveCommand(Claw claw, double ClawTargetPosition){
        this.ClawTargetPosition = ClawTargetPosition;
        this.claw = claw;

        addRequirements(claw);
    }
    @Override
    public void initialize(){
        claw.SetPos(ClawTargetPosition);
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
