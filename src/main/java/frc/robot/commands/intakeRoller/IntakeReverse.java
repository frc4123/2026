package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeReverse extends Command{

    IntakeRoller intakeRollers;
    IntakeArm intakeArm;

    public IntakeReverse(IntakeRoller intakeRollers, IntakeArm intakeArm) {
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
        addRequirements(intakeRollers);
    }

    @Override
    public void execute() {
        if(intakeArm.getIntakePosition() <= 0.115) {
            intakeRollers.setIntakeVelo(IntakeRollerConstants.reverseVelo);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }
}