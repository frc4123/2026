package frc.robot.commands.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollerStop extends Command {

    IntakeRoller intakeRollers;

    public IntakeRollerStop(final IntakeRoller intakeRollers) {
        this.intakeRollers = intakeRollers;
        this.addRequirements(intakeRollers);
    }

    @Override
    public void execute() {
        this.intakeRollers.setIntakeVelo(IntakeRollerConstants.ZERO_VELO);
    }
}
