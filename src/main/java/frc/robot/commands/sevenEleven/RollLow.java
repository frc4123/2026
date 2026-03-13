package frc.robot.commands.sevenEleven;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;

public class RollLow extends Command{

    SevenEleven sevenEleven;

    public RollLow(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenLowVelo);
    }
}