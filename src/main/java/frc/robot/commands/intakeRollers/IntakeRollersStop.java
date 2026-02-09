package frc.robot.commands.intakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeRollers;

public class IntakeRollersStop extends Command{

    IntakeRollers intakeRollers;

    public IntakeRollersStop(IntakeRollers intakeRollers) {
        intakeRollers = this.intakeRollers;
    }

    @Override
    public void execute() {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }

}