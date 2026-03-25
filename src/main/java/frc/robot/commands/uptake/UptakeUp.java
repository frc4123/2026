package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.SevenEleven;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShotHelper;

public class UptakeUp extends Command{

    Uptake uptake;
    Turret turret;
    SevenEleven sevenEleven;

    public UptakeUp(Uptake uptake, Turret turret, SevenEleven sevenEleven){ 
        this.uptake = uptake;
        this.turret = turret;
        this.sevenEleven = sevenEleven;
        addRequirements(uptake);
    }

    @Override
    public void execute() {
        if (ShotHelper.getIsWrapping()){
            uptake.zeroUptakeVelo();
            return;
        }
        uptake.setUptakeVelo(UptakeConstants.uptakeVelo);
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenHighVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        uptake.zeroUptakeVelo();
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.zeroVelo);
    }
}