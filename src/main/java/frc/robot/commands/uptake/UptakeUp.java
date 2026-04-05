package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShotHelper;

public class UptakeUp extends Command {

    Uptake uptake;
    Turret turret;
    SevenEleven sevenEleven;
    Shooter shooter;

    public UptakeUp(
            final Uptake uptake,
            final Turret turret,
            final SevenEleven sevenEleven,
            final Shooter shooter) {
        this.uptake = uptake;
        this.turret = turret;
        this.sevenEleven = sevenEleven;
        this.shooter = shooter;
        this.addRequirements(uptake);
    }

    @Override
    public void execute() {
        if (ShotHelper.getIsWrapping()) {
            this.uptake.zeroUptakeVelo();
            return;
        }
        this.shooter.setShooting(true);
        this.uptake.setUptakeVelo();
        this.sevenEleven.setSevenElevenVelo(SevenElevenConstants.SEVEN_ELEVEN_HIGH_VELO);
    }

    @Override
    public void end(final boolean interrupted) {
        this.uptake.zeroUptakeVelo();
        this.shooter.setShooting(false);
        this.sevenEleven.setSevenElevenVelo(SevenElevenConstants.ZERO_VELO);
    }
}
