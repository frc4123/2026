package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbDown extends Command{

    Climb climb;

    public ClimbDown(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setClimbPosition(ClimbConstants.downPosition);
    }
}