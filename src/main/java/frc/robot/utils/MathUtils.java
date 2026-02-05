package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MathUtils {

    public static boolean withinTolerance(Rotation2d value, Rotation2d tolerance) {
        return withinTolerance(value.getDegrees(), tolerance.getDegrees());
    }

    public static boolean withinTolerance(double value, double tolerance) {
        return Math.abs(value) <= Math.abs(tolerance);
    }

    public static Pose2d findClosestTarget(Pose2d current, Pose2d[] targets, boolean isRedAlliance) {
        if (current == null) {
            return null;
        }
        if (targets == null) {
            throw new IllegalArgumentException("Target list cannot be null or empty.");
        }
    
        Pose2d closest = null;
        double minDistance = Double.MAX_VALUE;
    
        for (Pose2d target : targets) {
            // Add logic to filter targets based on alliance if needed
            // For example, if targets have an alliance-specific property, check it here
            // For now, assume targets are already filtered by alliance
            double distance = current.getTranslation().getDistance(target.getTranslation());
            if (distance <= minDistance) {
                minDistance = distance;
                closest = target;
            }
        }
    
        return closest;
    }

    
}