package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeArmMid extends Command{

    IntakeArm intakeArm;
    IntakeRoller intakeRoller;

    public IntakeArmMid(IntakeArm intakeArm, IntakeRoller intakeRoller) {
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        if (intakeRoller.isIntaking()) {return;}
        intakeArm.setIntakePosition(IntakeArmConstants.midPosition);
    }
    
    @Override
    public void end(boolean interrupted) {
        //intakeArm.setIntakePosition(IntakeArmConstants.stowPosition);
    }
}