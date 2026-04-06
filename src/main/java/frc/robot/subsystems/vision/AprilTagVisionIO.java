package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false; // NOSONAR
        public AprilTagPoseObservation[] poseObservations = // NOSONAR
                new AprilTagPoseObservation[0];
        public int[] tagIds = new int[0]; // NOSONAR
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation. */
    public static record AprilTagPoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance) {}

    default void updateInputs(final VisionIOInputs inputs) {}
}
