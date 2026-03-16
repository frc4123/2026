package frc.robot.commands.intakeRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class AutoIntakeRollerIn extends Command{

    IntakeRoller intakeRollers;
    IntakeArm intakeArm;

    public AutoIntakeRollerIn(IntakeRoller intakeRollers, IntakeArm intakeArm) {
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
        addRequirements(intakeRollers);
    }// TODO MAKE CHECK THE STATE OF THE ARM BEFORE ROLLING

    @Override
    public void execute() {
        if(intakeArm.getIntakePosition() <= 0.115) {
            intakeRollers.setIntakeVelo(IntakeRollerConstants.intakeVelo);
        } else {
            intakeRollers.setIntakeVelo(0);
        }
    }
}