package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;

public class IntakeArmIn extends Command{

    IntakeArm intakeArm;
    boolean stalled;

    public IntakeArmIn(IntakeArm intakeArm) {
        this.intakeArm = intakeArm;
        addRequirements(intakeArm);
    }

    @Override
    public boolean isFinished(){
        stalled = intakeArm.getCurrent() > IntakeArmConstants.currentCancelationThreshold;
        return stalled;
    }

    @Override
    public void execute() {
        intakeArm.setIntakePosition(IntakeArmConstants.stowPosition);
    }

     @Override
    public void end(boolean interrupted) {
        if (stalled) {
            intakeArm.setBrakeMode();
        } else {
           intakeArm.setCoastMode();
        }
    }
}