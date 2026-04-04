package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbDown extends Command {

    Climb climb;

    public ClimbDown(final Climb climb) {
        this.climb = climb;
        this.addRequirements(climb);
    }

    @Override
    public void execute() {
        this.climb.setClimbPosition(ClimbConstants.DOWN_POSITION);
    }
}
