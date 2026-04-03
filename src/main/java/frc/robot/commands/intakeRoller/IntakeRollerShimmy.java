package frc.robot.commands.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollerShimmy extends Command {

    IntakeRoller intakeRollers;
    IntakeArm intakeArm;

    public IntakeRollerShimmy(final IntakeRoller intakeRollers, final IntakeArm intakeArm) {
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
    }// TODO MAKE CHECK THE STATE OF THE ARM BEFORE ROLLING

    @Override
    public void execute() {
        if (this.intakeRollers.isIntaking())
            return; // full speed command is running, don't interfere
        if (this.intakeArm.getIntakePosition() <= 0.115) {
            this.intakeRollers.setIntakeVelo(IntakeRollerConstants.INTAKE_VELO * 0.2);
        }
    }

    @Override
    public void end(final boolean interrupted) {
        this.intakeRollers.setIntakeVelo(IntakeRollerConstants.ZERO_VELO);
    }
}