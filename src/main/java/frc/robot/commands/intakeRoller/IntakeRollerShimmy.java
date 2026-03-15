package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollerShimmy extends Command{

    IntakeRoller intakeRollers;
    IntakeArm intakeArm;

    public IntakeRollerShimmy(IntakeRoller intakeRollers, IntakeArm intakeArm) {
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
    }// TODO MAKE CHECK THE STATE OF THE ARM BEFORE ROLLING

    @Override
    public void execute() {
        if (intakeRollers.isIntaking()) return; // full speed command is running, don't interfere
        if (intakeArm.getIntakePosition() <= 0.115) {
            intakeRollers.setIntakeVelo(IntakeRollerConstants.intakeVelo / 2);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }
}