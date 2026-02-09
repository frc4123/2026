package frc.robot.commands.intakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeRollers;

public class IntakeIn extends Command{

    IntakeRollers intakeRollers;

    public IntakeIn(IntakeRollers intakeRollers) {
        intakeRollers = this.intakeRollers;
    }

    @Override
    public void execute() {
        intakeRollers.setIntakeVelo(IntakeConstants.intakeVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeRollers.setIntakeVelo(IntakeConstants.zeroVelo);
    }
}