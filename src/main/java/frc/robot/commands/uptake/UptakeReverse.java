package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Uptake;

public class UptakeReverse extends Command {

    Uptake uptake;

    public UptakeReverse(final Uptake uptake) {
        this.uptake = uptake;
        this.addRequirements(uptake);
    }

    @Override
    public void execute() {
        this.uptake.setUptakeVelo();
    }
}
