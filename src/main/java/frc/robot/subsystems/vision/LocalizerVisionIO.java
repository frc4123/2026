package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LocalizerVisionIO {
    @AutoLog
    public static class LocalizerIOInputs {
        public boolean connected = false; // NOSONAR
        public LocalizerPoseObservation[] poseObservations = // NOSONAR
                new LocalizerPoseObservation[0];
    }

    public static record LocalizerPoseObservation(double timestamp, Pose3d pose) {}

    // A vision scheme that doesn't rely on tags will not know where it is on the field, and thus
    // will need to be inited to some known pose. This can be done during auto.
    default void setRobotPose(final Pose3d robotPose) {}

    default void updateInputs(final LocalizerIOInputs inputs) {}
}
