package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;


public class HoodAim extends Command {
    private final Hood hood;

    public HoodAim(Hood hood){
        this.hood = hood;
        addRequirements(hood);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void execute(){
        hood.setHoodAngle();
    }
}




