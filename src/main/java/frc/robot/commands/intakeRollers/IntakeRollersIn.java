package frc.robot.commands.intakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollersIn extends Command{

    IntakeRoller intakeRollers;

    public IntakeRollersIn(IntakeRoller intakeRollers) {
        intakeRollers = this.intakeRollers;
    }

    @Override
    public void execute() {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.intakeVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }
}