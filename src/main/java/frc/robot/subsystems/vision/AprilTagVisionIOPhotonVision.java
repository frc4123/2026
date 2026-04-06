package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class AprilTagVisionIOPhotonVision implements AprilTagVisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public AprilTagVisionIOPhotonVision(
            final String name,
            final Transform3d robotToCamera,
            final AprilTagFieldLayout fieldLayout) {
        this.camera = new PhotonCamera(name);
        this.estimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    }

    @Override
    public void updateInputs(final VisionIOInputs inputs) {
        inputs.connected = this.camera.isConnected();
        final List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();

        final Set<Integer> tagIds = new HashSet<>();
        final List<AprilTagPoseObservation> poseObservations = new ArrayList<>();
        for (final PhotonPipelineResult result : results) {
            final var estPoseOpt = this.estimator.estimateLowestAmbiguityPose(result);
            if (!result.hasTargets() || estPoseOpt.isEmpty()) {
                continue;
            }
            final EstimatedRobotPose estPose = estPoseOpt.get();
            double totalTagDistance = 0.0;
            for (final var target : estPose.targetsUsed) {
                tagIds.add(target.getFiducialId());
                totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            final double avgDistance =
                    estPose.targetsUsed.isEmpty()
                            ? 0.0
                            : totalTagDistance / estPose.targetsUsed.size();

            poseObservations.add(
                    new AprilTagPoseObservation(
                            estPose.timestampSeconds,
                            estPose.estimatedPose,
                            result.getBestTarget().getPoseAmbiguity(),
                            estPose.targetsUsed.size(),
                            avgDistance));
        }
        // Convert our List to an array of type PoseObservation
        inputs.poseObservations = poseObservations.toArray(new AprilTagPoseObservation[0]);
        // Unbox our Set
        inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
    }
}
