package frc.robot.utils.Factories;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Field;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FeedTargetFactory {

    private static CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    static InterpolatingTreeMap<Double, Double> distanceOffsetMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    static {
        distanceOffsetMap.put(0.0, Units.inchesToMeters(0.0));
    }

    static Double kXDistanceOffset = Units.inchesToMeters(0);

    public static boolean inFieldLeft() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return swerve.getState().Pose.getY() >= halfWidth;
    }

    public static boolean inFieldRight() {
        final double fieldWidthMeters = Units.feetToMeters(27.0); // full field width (Y)
        final double halfWidth = fieldWidthMeters / 2.0;

        return swerve.getState().Pose.getY() < halfWidth;
    }

    public static Translation2d generate() {
        boolean inFieldLeft = inFieldLeft();
        Translation2d feedTarget;

        if (inFieldLeft) {
            feedTarget = Field.isBlue() ? Field.LeftBlueBump.centerPose
                    : Field.BlueToRed(Field.LeftBlueBump.centerPose);
        } else {
            feedTarget = Field.isBlue() ? Field.RightBlueBump.centerPose
                    : Field.BlueToRed(Field.RightBlueBump.centerPose);
        }

        double distance = new Translation2d(feedTarget.getX(), feedTarget.getY()).getDistance(swerve.getState().Pose.getTranslation());
        double distanceOffset = distanceOffsetMap.get(distance);
        // Do math in blue alliance, we flip for red.
        var offSet = new Translation2d(kXDistanceOffset, -distanceOffset);

        if (Field.isRed()) {
            offSet = new Translation2d(-offSet.getX(), offSet.getY());
        }

        feedTarget = new Translation2d(
                feedTarget.getX() + offSet.getX(), 
                feedTarget.getY() + offSet.getY());
        return feedTarget;
    }
}