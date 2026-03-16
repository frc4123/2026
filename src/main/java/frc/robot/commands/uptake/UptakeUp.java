package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShotHelper;

public class UptakeUp extends Command{

    Uptake uptake;
    Turret turret;

    public UptakeUp(Uptake uptake, Turret turret){ 
        this.uptake = uptake;
        this.turret = turret;
        addRequirements(uptake);
    }

    @Override
    public void execute() {
        if (ShotHelper.getIsWrapping()){
            uptake.setUptakeVelo(UptakeConstants.zeroVelo);
            return;
        }
        uptake.setUptakeVelo(UptakeConstants.uptakeVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        uptake.setUptakeVelo(UptakeConstants.zeroVelo);
    }
}