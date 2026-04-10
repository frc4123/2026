package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Target {

    private static CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public static Translation3d getTarget() {

        final double x = swerve.getState().Pose.getX();
        final double y = swerve.getState().Pose.getY();

        if (Field.isBlue()) {
            if (x < Constants.VisionConstants.BLUE_HUB.getX()) {
            } else if (y >= 5.029) {
                // Top zone - depot
                return Constants.VisionConstants.blueDepot.getTranslation(); // blueDepot Convert
                // Pose2d to
                // Translation3d
            } else if (y > 4.044) {
                // Upper middle zone - left bump corner
                return Constants.VisionConstants.blueDepot.getTranslation(); // blueLeftBumpCorner.
            } else if (y > 3.059) {
                // Lower middle zone - right bump corner
                if (DriverStation.isAutonomous()) {
                    return VisionConstants.blueAutoAimThreshold.getTranslation();
                }
                return Constants.VisionConstants.BLUE_AIM_THRESHOLD
                        .getTranslation(); // blueRIghtBumpCorner
            } else {
                // Bottom zone - aim threshold
                if (DriverStation.isAutonomous()) {
                    return VisionConstants.blueAutoAimThreshold.getTranslation();
                } // TODO make for red side and left side too same for all autos
                return Constants.VisionConstants.BLUE_AIM_THRESHOLD
                        .getTranslation(); // blueAimThreshold
            }
        } else if (Field.isRed()) {
            if (x > Constants.VisionConstants.RED_HUB.getX()) {
                return Constants.VisionConstants.redHubTranslation3d;
                // Past hub - match the Y zones from Turret.targetAngle()
            } else if (y >= 5.029) {
                // Top zone - aim threshold

                if (DriverStation.isAutonomous()) {
                    return VisionConstants.redAutoAimThreshold.getTranslation();
                }
                return Constants.VisionConstants.RED_AIM_THRESHOLD
                        .getTranslation(); // redAimThreshold
            } else if (y > 4.044) {
                // Upper middle zone - right bump corner
                if (DriverStation.isAutonomous()) {
                    return VisionConstants.redAutoAimThreshold.getTranslation();
                }
                return Constants.VisionConstants.RED_AIM_THRESHOLD
                        .getTranslation(); // redRightBumpCorner
            } else if (y > 3.059) {
                // Lower middle zone - left bump corner
                return Constants.VisionConstants.redDepot.getTranslation(); // redLeftBumpCorner
            } else {
                // Bottom zone - depot
                return Constants.VisionConstants.redDepot.getTranslation(); // redDepot
            }
        }

        return Constants.VisionConstants.blueHubTranslation3d;
    }

    public static Rotation2d getTrenchAngle(final double x) {
        if (Field.isBlue()) {
            if (x > 4.63) {
                return new Rotation2d(Math.toRadians(0));
            } else {
                return new Rotation2d(Math.toRadians(180));
            }
        } else {
            if (x > 11.91) {
                return new Rotation2d(Math.toRadians(180));
            } else {
                return new Rotation2d(Math.toRadians(0));
            }
        }
    }

    public static Rotation2d getBumpAngle(final double x) {
        if (Field.isBlue()) {
            if (x > 4.63) {
                return new Rotation2d(Math.toRadians(135));
            } else {
                return new Rotation2d(Math.toRadians(45));
            }
        } else {
            if (x > 11.91) {
                return new Rotation2d(Math.toRadians(45));
            } else {
                return new Rotation2d(Math.toRadians(135));
            }
        }
    }
}
