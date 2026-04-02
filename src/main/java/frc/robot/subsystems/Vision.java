package frc.robot.subsystems;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {

    public static PhotonPipelineResult getLatestResults(final PhotonCamera camera) {
        final List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return null;
        }

        final PhotonPipelineResult latest = results.get(results.size() - 1);

        if (!latest.hasTargets()) {
            return null;
        }

        return latest;
    }

    public static AprilTagFieldLayout loadAprilTagFieldLayout(final String resourceFile) {
        try (InputStream is = Vision.class.getResourceAsStream(resourceFile);
                InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) {
            return new ObjectMapper().readValue(isr, AprilTagFieldLayout.class);
        } catch (final IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    // private final StructPublisher<Pose3d> CamPosePublisher;
    // private final StructPublisher<Transform3d> CamTargetTransformPublisher;

    private final AprilTagFieldLayout aprilTagFieldLayout;
    // Standard deviations (tune these based on camera characteristics)
    // third parameter should be double the first 2
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    // the higher the number the less you trust your camera additions
    private int camProcessorCounter = 0;

    // This is too much voodoo
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    private final Transform3d FLO_robotToCam;
    private final Transform3d FLI_robotToCam;
    private final Transform3d FRI_robotToCam;
    private final Transform3d FRO_robotToCam;

    private final PhotonCamera FLO_camera = new PhotonCamera("Front_Left_Outside");
    private final PhotonCamera FLI_camera = new PhotonCamera("Front_Left_Inside");
    private final PhotonCamera FRI_camera = new PhotonCamera("Front_Right_Inside");
    private final PhotonCamera FRO_camera = new PhotonCamera("Front_Right_Outside");

    private final PhotonPoseEstimator FLO_Estimator;
    private final PhotonPoseEstimator FLI_Estimator;
    private final PhotonPoseEstimator FRI_Estimator;
    private final PhotonPoseEstimator FRO_Estimator;

    public Vision() {
        this.aprilTagFieldLayout = Vision.loadAprilTagFieldLayout("/fields/2026Welded.json");

        // Camera transforms
        this.FLO_robotToCam = new Transform3d(
                new Translation3d(
                        VisionConstants.FLO_frontX,
                        VisionConstants.FLO_frontY,
                        VisionConstants.FLO_frontZ),
                new Rotation3d(
                        VisionConstants.FLO_frontRoll,
                        VisionConstants.FLO_frontPitch,
                        VisionConstants.FLO_frontYaw));

        this.FLI_robotToCam = new Transform3d(
                new Translation3d(
                        VisionConstants.FLI_frontX,
                        VisionConstants.FLI_frontY,
                        VisionConstants.FLI_frontZ),
                new Rotation3d(
                        VisionConstants.FLI_frontRoll,
                        VisionConstants.FLI_frontPitch,
                        VisionConstants.FLI_frontYaw));

        this.FRI_robotToCam = new Transform3d(
                new Translation3d(
                        VisionConstants.FRI_frontX,
                        VisionConstants.FRI_frontY,
                        VisionConstants.FRI_frontZ),
                new Rotation3d(
                        VisionConstants.FRI_frontRoll,
                        VisionConstants.FRI_frontPitch,
                        VisionConstants.FRI_frontYaw));

        this.FRO_robotToCam = new Transform3d(
                new Translation3d(
                        VisionConstants.FRO_frontX,
                        VisionConstants.FRO_frontY,
                        VisionConstants.FRO_frontZ),
                new Rotation3d(
                        VisionConstants.FRO_frontRoll,
                        VisionConstants.FRO_frontPitch,
                        VisionConstants.FRO_frontYaw));

        // front camera estimator (new 2026 syntax)
        this.FLO_Estimator = new PhotonPoseEstimator(
                this.aprilTagFieldLayout,
                this.FLO_robotToCam);

        this.FLI_Estimator = new PhotonPoseEstimator(
                this.aprilTagFieldLayout,
                this.FLI_robotToCam);

        this.FRI_Estimator = new PhotonPoseEstimator(
                this.aprilTagFieldLayout,
                this.FRI_robotToCam);

        this.FRO_Estimator = new PhotonPoseEstimator(
                this.aprilTagFieldLayout,
                this.FRO_robotToCam);

        // Initialize NetworkTables publishers
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // CamPosePublisher = inst.getStructTopic("/Vision/LeftCameraPose",
        // Pose3d.struct).publish();
        // CamTargetTransformPublisher =
        // inst.getStructTopic("/Vision/LeftCamTargetTransform",
        // Transform3d.struct).publish();

    }

    @Override
    public void periodic() {
        switch (this.camProcessorCounter % 4) { // NOSONAR
            case 0:
                this.processVision(this.FLO_camera, this.FLO_Estimator);
                break;
            case 1:
                this.processVision(this.FLI_camera, this.FLI_Estimator);
                break;
            case 2:
                this.processVision(this.FRI_camera, this.FRI_Estimator);
                break;
            case 3:
                this.processVision(this.FRO_camera, this.FRO_Estimator);
                break;
        }
        this.camProcessorCounter++;
    }

    private void processVision(final PhotonCamera camera, final PhotonPoseEstimator estimator) {

        final PhotonPipelineResult result = Vision.getLatestResults(camera);
        if (result == null)
            return;

        final List<PhotonTrackedTarget> validTargets = this.getValidTargets(result);

        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        final PhotonPipelineResult filteredResult = new PhotonPipelineResult(
                result.metadata,
                validTargets,
                Optional.empty());

        if (validTargets.size() > 1) {

            estimatedPose = estimator.estimateCoprocMultiTagPose(filteredResult);

        } else {
            final Optional<EstimatedRobotPose> singleTagPose = estimator.estimateLowestAmbiguityPose(filteredResult);

            if (singleTagPose.isPresent()) {
                final PhotonTrackedTarget best = result.getBestTarget();
                if (best != null &&
                        best.getPoseAmbiguity() < VisionConstants.ambiguityThreshold) {
                    estimatedPose = singleTagPose;
                }
            }
        }

        if (estimatedPose.isPresent()) {
            final EstimatedRobotPose est = estimatedPose.get();

            final Matrix<N3, N1> stdDevs = this.calculateStdDevs(est, validTargets);

            this.swerve.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    stdDevs);
        }
    }

    private List<PhotonTrackedTarget> getValidTargets(final PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> validTargets = new ArrayList<>();
        for (final PhotonTrackedTarget target : result.getTargets()) {

            // this is reallllly tiny
            if (target.getPoseAmbiguity() > VisionConstants.ambiguityThreshold) {
                continue;
            }

            validTargets.add(target);
        }

        return validTargets;
    }

    private Matrix<N3, N1> calculateStdDevs(final EstimatedRobotPose est, final List<PhotonTrackedTarget> targets) {
        int numTags = 0;
        double totalDistance = 0;

        for (final PhotonTrackedTarget target : targets) {
            final Optional<Pose3d> tagPose = this.aprilTagFieldLayout.getTagPose(target.getFiducialId());
            // Reject low-ambiguity threshold — tune this
            final var hasHighAmbiguity = target.getPoseAmbiguity() > VisionConstants.ambiguityThreshold;
            if (tagPose.isEmpty() || hasHighAmbiguity)
                continue;

            numTags++;
            totalDistance += tagPose.get().toPose2d().getTranslation()
                    .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0)
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        final double avgDistance = totalDistance / numTags;

        // Hard reject if too far — tags beyond 4m are unreliable
        if (avgDistance > 4.0)
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        final Matrix<N3, N1> baseDevs = this.singleTagStdDevs;

        // Aggressive cubic scaling instead of quadratic
        double scalar = 1 + (avgDistance * avgDistance * avgDistance / 15.0);

        // extra penalty
        scalar *= 3.0;

        return baseDevs.times(scalar);
    }
}