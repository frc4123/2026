package frc.robot.commands.sevenEleven;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;

public class Roll extends Command{

    SevenEleven sevenEleven;

    public Roll(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.zeroVelo);
    }
}