package frc.robot.commands.intakeArm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeShimmy extends Command {

    private final IntakeArm intakeArm;
    private final IntakeRoller intakeRoller;
    private Command cycle;

    public IntakeShimmy(IntakeArm intakeArm, IntakeRoller intakeRoller) {
        this.intakeArm = intakeArm;
        this.intakeRoller = intakeRoller;
        addRequirements(intakeArm);
    }

    @Override
    public void execute() {
        boolean currentlyIntaking = intakeRoller.isIntaking();

        // If intake starts, cancel shimmy and set arm out
        if (currentlyIntaking) {
            if (cycle != null && cycle.isScheduled()) {
                cycle.cancel();
                cycle = null;
            }
            intakeArm.setIntakePosition(IntakeArmConstants.outPosition);
            return;
        }

        // If not intaking, schedule repeating shimmy if not already running
        if (cycle == null || !cycle.isScheduled()) {
            cycle = new RepeatCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> intakeArm.setIntakePosition(IntakeArmConstants.stowPosition), intakeArm),
                    new WaitCommand(0.7),
                    new InstantCommand(() -> intakeArm.setIntakePosition(IntakeArmConstants.outPosition), intakeArm),
                    new WaitCommand(0.7)
                )
            );
            CommandScheduler.getInstance().schedule(cycle);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Never finishes on its own
    }
}