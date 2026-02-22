package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command{

    Climb climb;

    public ClimbUp(Climb climb) {
        this.climb = climb;
    }

    @Override
    public void execute() {
        climb.setClimbPosition(ClimbConstants.upPosition);
    }
}