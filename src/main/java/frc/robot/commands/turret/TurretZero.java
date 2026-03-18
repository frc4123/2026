package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;

public class TurretZero extends Command {
    private final Turret turret;
    private final CommandSwerveDrivetrain drivetrain;

    public TurretZero(Turret turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void execute() {
        turret.setFieldAngle(
            turret.targetAngle(drivetrain.getState().Pose)
        );
    }
}
