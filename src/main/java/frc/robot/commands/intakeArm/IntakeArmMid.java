package frc.robot.commands.intakearm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeArmMid extends Command {

    IntakeArm intakeArm;
    IntakeRoller intakeRoller;

    public IntakeArmMid(final IntakeArm intakeArm, final IntakeRoller intakeRoller) {
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        this.addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        if (this.intakeRoller.isIntaking()) {
            return;
        }
        this.intakeArm.setIntakePosition(IntakeArmConstants.MID_POSITION);
    }

    @Override
    public void end(final boolean interrupted) {
        // intakeArm.setIntakePosition(IntakeArmConstants.stowPosition);
    }
}
