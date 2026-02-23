package frc.robot.utils;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;

public class ShotCache {

    private static ShotData cachedShot;
    private static final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static void update() {
        cachedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
            swerve.getState().Pose,
            swerve.getState().Speeds,
            Target.getTarget(),
            3
        );
    }

    public static ShotData get() {
        return cachedShot;
    }
}