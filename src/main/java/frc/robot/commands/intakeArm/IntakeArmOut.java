package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmOut extends Command {

    IntakeArm intakeArm;

    public IntakeArmOut(final IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        this.addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        this.intakeArm.setIntakePosition(IntakeArmConstants.OUT_POSITION);
    }

    @Override
    public void end(final boolean interrupted) {
        // intakeArm.setIntakePosition(IntakeArmConstants.stowPosition);
    }
}