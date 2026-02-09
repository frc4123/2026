package frc.robot.commands.intakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeRollers;

public class IntakeStop extends Command{

    IntakeRollers intakeRollers;

    public IntakeStop(IntakeRollers intakeRollers) {
        intakeRollers = this.intakeRollers;
    }

    @Override
    public void execute() {
        intakeRollers.setIntakeVelo(IntakeConstants.zeroVelo);
    }

}