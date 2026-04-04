package frc.robot.subsystems;

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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.Field;
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
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
    // the higher the number the less you trust your camera additions
    private int camProcessorCounter = 0;

    // This is too much voodoo
    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    private final Transform3d floRobotToCam;
    private final Transform3d fliRobotToCam;
    private final Transform3d friRobotToCam;
    private final Transform3d froRobotToCam;

    private final PhotonCamera floCamera = new PhotonCamera("Front_Left_Outside");
    private final PhotonCamera fliCamera = new PhotonCamera("Front_Left_Inside");
    private final PhotonCamera friCamera = new PhotonCamera("Front_Right_Inside");
    private final PhotonCamera froCamera = new PhotonCamera("Front_Right_Outside");

    private final PhotonPoseEstimator floEstimator;
    private final PhotonPoseEstimator fliEstimator;
    private final PhotonPoseEstimator friEstimator;
    private final PhotonPoseEstimator froEstimator;

    public Vision() {
        this.aprilTagFieldLayout = Vision.loadAprilTagFieldLayout("/fields/2026Welded.json");

        // Camera transforms
        this.floRobotToCam =
                new Transform3d(
                        new Translation3d(
                                VisionConstants.FLO_X,
                                VisionConstants.FLO_Y,
                                VisionConstants.FLO_Z),
                        new Rotation3d(
                                VisionConstants.FLO_ROLL,
                                VisionConstants.FLO_PITCH,
                                VisionConstants.FLO_YAW));

        this.fliRobotToCam =
                new Transform3d(
                        new Translation3d(
                                VisionConstants.FLI_X,
                                VisionConstants.FLI_Y,
                                VisionConstants.FLI_Z),
                        new Rotation3d(
                                VisionConstants.FLI_ROLL,
                                VisionConstants.FLI_PITCH,
                                VisionConstants.FLI_YAW));

        this.friRobotToCam =
                new Transform3d(
                        new Translation3d(
                                VisionConstants.FRI_X,
                                VisionConstants.FRI_Y,
                                VisionConstants.FRI_Z),
                        new Rotation3d(
                                VisionConstants.FRI_ROLL,
                                VisionConstants.FRI_PITCH,
                                VisionConstants.FRI_YAW));

        this.froRobotToCam =
                new Transform3d(
                        new Translation3d(
                                VisionConstants.FRO_X,
                                VisionConstants.FRO_Y,
                                VisionConstants.FRO_Z),
                        new Rotation3d(
                                VisionConstants.FRO_ROLL,
                                VisionConstants.FRO_PITCH,
                                VisionConstants.FRO_YAW));

        // front camera estimator (new 2026 syntax)
        this.floEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, this.floRobotToCam);

        this.fliEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, this.fliRobotToCam);

        this.friEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, this.friRobotToCam);

        this.froEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, this.froRobotToCam);

        // Initialize NetworkTables publishers
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // CamPosePublisher = inst.getStructTopic("/Vision/LeftCameraPose",
        // Pose3d.struct).publish();
        // CamTargetTransformPublisher =
        // inst.getStructTopic("/Vision/LeftCamTargetTransform",
        // Transform3d.struct).publish();

    }

    private void processVision(final PhotonCamera camera, final PhotonPoseEstimator estimator) {

        final PhotonPipelineResult result = getLatestResults(camera);
        if (result == null) return;

        final List<PhotonTrackedTarget> validTargets = this.getValidTargets(result, estimator);

        // if (validTargets.isEmpty()) return;

        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

        /*
         * if (result.getMultiTagResult().isPresent() && validTargets.size() > 1) {
         * estimatedPose = estimator.estimateCoprocMultiTagPose(result);
         * } else {
         */
        if (result.hasTargets()) {
            final var tagId = result.getBestTarget().getFiducialId();
            if (!this.isTagOurHub(tagId)) {
                return;
            }
        } else {
            // I don't think this logic was reached before, but i think it's what is
            // intended
            return;
        }

        final Optional<EstimatedRobotPose> singleTagPose =
                estimator.estimateLowestAmbiguityPose(result);

        if (singleTagPose.isPresent()) {
            final PhotonTrackedTarget best = result.getBestTarget();

            if (best != null && best.getPoseAmbiguity() < VisionConstants.AMBIGUITY_THRESHOLD) {
                estimatedPose = singleTagPose;
            }
        }
        // }

        if (estimatedPose.isPresent()) {
            final EstimatedRobotPose est = estimatedPose.get();

            final Matrix<N3, N1> stdDevs = this.calculateStdDevs(est, validTargets);

            this.swerve.addVisionMeasurement(
                    est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs);
        }
    }

    private List<PhotonTrackedTarget> getValidTargets(
            final PhotonPipelineResult result, final PhotonPoseEstimator estimator) {
        final List<PhotonTrackedTarget> validTargets = new ArrayList<>();
        for (final PhotonTrackedTarget target : result.getTargets()) {

            // this is reallllly tiny
            if (target.getPoseAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD) {
                continue;
            }

            validTargets.add(target);
        }

        return validTargets;
    }

    private Matrix<N3, N1> calculateStdDevs(
            final EstimatedRobotPose est, final List<PhotonTrackedTarget> targets) {
        int numTags = 0;
        double totalDistance = 0;

        for (final PhotonTrackedTarget target : targets) {
            final Optional<Pose3d> tagPose =
                    this.aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()
                    || target.getPoseAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD) continue;

            numTags++;
            totalDistance +=
                    tagPose.get()
                            .toPose2d()
                            .getTranslation()
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

    public boolean isTagOurHub(final int tagId) {
        // if (result == null) {
        // return false;
        // }
        // final int tagId = result.getBestTarget().getFiducialId();
        final var blueBlue = FieldConstants.BLUE_HUB_TAG_IDS.contains(tagId) && Field.isBlue();
        final var redRed = FieldConstants.RED_HUB_TAG_IDS.contains(tagId) && Field.isRed();
        return blueBlue || redRed;
    }

    @Override
    public void periodic() {
        switch (this.camProcessorCounter % 4) { // NOSONAR
            case 0:
                this.processVision(this.floCamera, this.floEstimator);
                break;
            case 1:
                this.processVision(this.fliCamera, this.fliEstimator);
                break;
            case 2:
                this.processVision(this.friCamera, this.friEstimator);
                break;
            case 3:
                this.processVision(this.froCamera, this.froEstimator);
                break;
        }
        this.camProcessorCounter++;
    }
}
