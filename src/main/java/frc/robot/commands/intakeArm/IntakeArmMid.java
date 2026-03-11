package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmMid extends Command{

    IntakeArm intakeArm;

    public IntakeArmMid(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        intakeArm.setIntakePosition(IntakeArmConstants.midPosition);
        intakeArm.setBrakeMode();
    }
    
    @Override
    public void end(boolean interrupted) {
        //intakeArm.setIntakePosition(IntakeArmConstants.stowPosition);
    }
}