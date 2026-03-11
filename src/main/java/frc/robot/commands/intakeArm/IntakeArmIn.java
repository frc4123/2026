package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeArmIn extends Command{

    IntakeArm intakeArm;
    IntakeRoller intakeRoller;
    boolean stalled;

    public IntakeArmIn(IntakeArm intakeArm, IntakeRoller intakeRoller) {
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        addRequirements(intakeArm);
    }

    @Override
    public boolean isFinished(){
        stalled = intakeArm.getCurrent() > IntakeArmConstants.currentCancelationThreshold;
        return stalled;
    }

    @Override
    public void execute() {
        if (intakeRoller.isIntaking()) {return;}
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