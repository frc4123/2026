package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class SetShooterVelocity extends Command {
    private final Shooter shooter;

    public SetShooterVelocity(Shooter shooter){
        this.shooter= shooter;
        addRequirements(shooter);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void execute(){
        shooter.setShooterVelo();
    }
}




