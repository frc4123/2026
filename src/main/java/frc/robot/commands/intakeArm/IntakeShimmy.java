package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeShimmy extends Command {

    private final IntakeArm intakeArm;
    private final IntakeRoller intakeRoller;
    private SequentialCommandGroup cycle;
    private boolean isIntaking = true;

    public IntakeShimmy(IntakeArm intakeArm, IntakeRoller intakeRoller) {
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        isIntaking = intakeRoller.isIntaking();
        if (!isIntaking) {
            if (cycle == null || !cycle.isScheduled()) {
                cycle = new SequentialCommandGroup(
                    new InstantCommand(() -> intakeArm.setIntakePosition(IntakeArmConstants.stowPosition), intakeArm),
                    new WaitCommand(0.9),
                    new InstantCommand(() -> intakeArm.setIntakePosition(IntakeArmConstants.outPosition), intakeArm),
                    new WaitCommand(0.9)
                );
                CommandScheduler.getInstance().schedule(cycle);

                if (cycle != null && !cycle.isScheduled()) {
                    cycle = null;
                }
            }
        } else if (isIntaking) {
            intakeArm.setIntakePosition(IntakeArmConstants.outPosition);

        } else {
            if (cycle != null && cycle.isScheduled()) {
                cycle.cancel();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}