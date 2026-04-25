package frc.robot.commands.seveneleven;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SevenEleven;

public class RollStop extends Command {

    SevenEleven sevenEleven;

    public RollStop(final SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        this.addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        this.sevenEleven.setSevenElevenVelo(0);
    }
}
