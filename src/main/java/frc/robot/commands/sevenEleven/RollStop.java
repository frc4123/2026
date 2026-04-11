package frc.robot.commands.sevenEleven;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SevenEleven;

public class RollStop extends Command{

    SevenEleven sevenEleven;

    public RollStop(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(0);
    }
}