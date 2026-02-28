package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.Uptake;

public class UptakeStop extends Command{

    Uptake uptake;

    public UptakeStop(Uptake uptake) {
        this.uptake = uptake;
        addRequirements(uptake);
    }

    @Override
    public void execute() {
        uptake.setUptakeVelo(UptakeConstants.zeroVelo);
    }

}