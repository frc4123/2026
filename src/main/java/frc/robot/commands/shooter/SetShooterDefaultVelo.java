package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class SetShooterDefaultVelo extends Command {
    private final Shooter shooter;

    public SetShooterDefaultVelo(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void execute(){
        shooter.shooterMinVelo();
    }
}




