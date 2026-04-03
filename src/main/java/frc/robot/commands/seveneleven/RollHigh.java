package frc.robot.commands.seveneleven;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;

public class RollHigh extends Command {

    SevenEleven sevenEleven;

    public RollHigh(final SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        this.addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        this.sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenHighVelo);
    }
}