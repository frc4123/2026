package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.Uptake;

public class UptakeUp extends Command{

    Uptake uptake;

    public UptakeUp(Uptake uptake) {
        this.uptake = uptake;
        addRequirements(uptake);
    }

    @Override
    public void execute() {
        uptake.setUptakeVelo(UptakeConstants.uptakeVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        uptake.setUptakeVelo(UptakeConstants.zeroVelo);
    }
}