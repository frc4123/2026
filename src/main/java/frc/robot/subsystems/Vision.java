package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        // I don't think this logic was reached before, but i think it's what is intended
        if (result == null || !result.hasTargets()) return;

        if (result.getBestTarget().getPoseAmbiguity() >= VisionConstants.AMBIGUITY_THRESHOLD) {
            return;
        }
        // TODO Again, I feel like this is weird to have to do. Perhaps wrap this in a shuffleboard
        // boolean and play around with the vision system
        if (!this.isTagOurHub(result.getBestTarget().getFiducialId())) {
            return;
        }

        final Optional<EstimatedRobotPose> estPoseOpt =
                estimator.estimateLowestAmbiguityPose(result);
        if (estPoseOpt.isEmpty()) {
            return;
        }
        final EstimatedRobotPose estPose = estPoseOpt.get();

        if (this.isEstOffField(estPose)) {
            return;
        }

        final List<PhotonTrackedTarget> validTargets = this.getValidTargets(result);
        final Matrix<N3, N1> stdDevs = this.calculateStdDevs(estPose, validTargets);

        // TODO Swerve should supply a consumer that we feed datapoints into--shouldn't have to
        // keep an instance in our class
        this.swerve.addVisionMeasurement(
                estPose.estimatedPose.toPose2d(), estPose.timestampSeconds, stdDevs);
    }

    public boolean isEstOffField(final EstimatedRobotPose est) {
        final double x = est.estimatedPose.getX();
        final double y = est.estimatedPose.getY();

        final boolean isOutsideX = (x < 0) || (x > Field.FIELD_LENGTH.in(Meters));
        final boolean isOutsideY = (y < 0) || (y > Field.FIELD_WIDTH.in(Meters));

        return isOutsideX || isOutsideY;
    }

    private List<PhotonTrackedTarget> getValidTargets(final PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> validTargets = new ArrayList<>();
        for (final PhotonTrackedTarget target : result.getTargets()) {
            final double area = target.getArea();
            // If the tag takes up less than (TAG_AREA_THRESHOLD *100)% of the screen, do not
            // consider it
            if ((target.getPoseAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD)
                    && (area < VisionConstants.TAG_AREA_THRESHOLD)) {
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
            if (tagPose.isEmpty()) continue;

            numTags++;
            totalDistance +=
                    tagPose.get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) return this.singleTagStdDevs;

        final double avgDistance = totalDistance / numTags;
        final Matrix<N3, N1> baseDevs = numTags >= 2 ? this.multiTagStdDevs : this.singleTagStdDevs;
        return baseDevs.times(0.2 + (avgDistance * avgDistance / 20));
    }

    public boolean isTagOurHub(final int tagId) {
        final var blueBlue = FieldConstants.BLUE_HUB_TAG_IDS.contains(tagId) && Field.isBlue();
        final var redRed = FieldConstants.RED_HUB_TAG_IDS.contains(tagId) && Field.isRed();
        return blueBlue || redRed;
    }

    public Rotation2d getTrenchAngle(final double x) {
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

    public int avoidDisconnectedCams(int camToChoose) {
        if (camToChoose == 0 && !this.floCamera.isConnected()) {
            camToChoose++;
        }
        if (camToChoose == 1 && !this.fliCamera.isConnected()) {
            camToChoose++;
        }
        if (camToChoose == 2 && !this.friCamera.isConnected()) {
            camToChoose++;
        }
        if (camToChoose == 3 && !this.froCamera.isConnected()) {
            camToChoose++;
        }

        return camToChoose;
    }

    @Override
    public void periodic() {
        int camToChoose = this.camProcessorCounter % 4;
        camToChoose = this.avoidDisconnectedCams(camToChoose);
        switch (camToChoose) {
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
            default:
                break;
        }
        this.camProcessorCounter++;
    }
}
