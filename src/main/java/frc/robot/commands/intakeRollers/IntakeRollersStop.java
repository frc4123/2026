package frc.robot.commands.intakeRollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollersStop extends Command{

    IntakeRoller intakeRollers;

    public IntakeRollersStop(IntakeRoller intakeRollers) {
        intakeRollers = this.intakeRollers;
    }

    @Override
    public void execute() {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }

}