package frc.robot.utils;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TrajectoryCalculator;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;

public class ShotCache {

    private static ShotData cachedShot = new ShotData(0, 60, VisionConstants.blueHubTranslation3d);
    private static final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static boolean isPassingShot() {
        final double robotX = swerve.getState().Pose.getX();

        if (Field.isBlue()) {
            return robotX > Constants.VisionConstants.BLUE_HUB.getX();
        } else if (Field.isRed()) {
            return robotX < Constants.VisionConstants.RED_HUB.getX();
        }

        return false;
    }

    public static void update() {
        if (!isPassingShot()) {
            cachedShot =
                    TrajectoryCalculator.iterativeMovingShotFromFunnelClearance(
                            swerve.getState().Pose,
                            swerve.getState().Speeds,
                            Target.getTarget(),
                            13 // was
                            // 9,
                            // 8,
                            // 6,
                            // 7, 3
                            );
        } else {
            cachedShot =
                    TrajectoryCalculator.iterativeMovingPass(
                            swerve.getState().Pose,
                            swerve.getState().Speeds,
                            Target.getTarget(),
                            13 // was 9, 8, 6, 7, 3
                            );
        }
    }

    public static ShotData get() {
        return cachedShot;
    }
}
