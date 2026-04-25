package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;

public class UptakeStop extends Command {

    Uptake uptake;
    Shooter shooter;

    public UptakeStop(final Uptake uptake, final Shooter shooter) {
        this.uptake = uptake;
        this.shooter = shooter;
        this.addRequirements(uptake);
    }

    @Override
    public void execute() {
        this.shooter.setShooting(false);
        this.uptake.zeroUptakeVelo();
    }
}
