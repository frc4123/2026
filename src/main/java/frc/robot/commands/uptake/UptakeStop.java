package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;

public class UptakeStop extends Command{

    Uptake uptake;
    Shooter shooter;

    public UptakeStop(Uptake uptake, Shooter shooter) {
        this.uptake = uptake;
        this.shooter = shooter;
        addRequirements(uptake);
    }

    @Override
    public void execute() {
        shooter.isShooting(false);
        uptake.zeroUptakeVelo();
    }

}