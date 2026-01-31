package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.turret.Turret;

public class Aim extends Command {
    private final Turret turret;
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    public Aim(Turret turret, CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(turret);
    }

    @Override
    public boolean isFinished() {
        return false; // Default commands should never finish on their own
    }

    @Override
    public void execute() {
        turret.setFieldAngle(
            turret.targetAngle(drivetrain.getState().Pose),
            vision.getTurretCamOffset()
        );
    }
}
