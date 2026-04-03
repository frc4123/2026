package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeArmInSlow extends Command {

    IntakeArm intakeArm;
    IntakeRoller intakeRoller;
    boolean stalled;

    public IntakeArmInSlow(final IntakeArm intakeArm, final IntakeRoller intakeRoller) {
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        this.addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        if (this.intakeRoller.isIntaking()) {
            return;
        }
        this.intakeArm.setSlowIntakePosition(IntakeArmConstants.STOW_POSITION);
    }
}