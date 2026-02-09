package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmIn extends Command{

    IntakeArm intakeArm;

    public IntakeArmIn(IntakeArm intakeArm) {
        intakeArm = this.intakeArm;
    }

    @Override
    public void execute() {
        intakeArm.setIntakePosition(IntakeArmConstants.stowPosition);
    }
}