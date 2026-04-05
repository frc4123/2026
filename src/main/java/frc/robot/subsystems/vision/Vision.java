package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.Field;

public class Vision extends SubsystemBase {

    // public static PhotonPipelineResult getLatestResults(final PhotonCamera camera) {
    // final List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    // if (results.isEmpty()) {
    // return null;
    // }

    // final PhotonPipelineResult latest = results.get(results.size() - 1);

    // if (!latest.hasTargets()) {
    // return null;
    // }

    // return latest;
    // }

    // public static AprilTagFieldLayout loadAprilTagFieldLayout(final String resourceFile) {
    // try (InputStream is = Vision.class.getResourceAsStream(resourceFile);
    // InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) {
    // return new ObjectMapper().readValue(isr, AprilTagFieldLayout.class);
    // } catch (final IOException e) {
    // throw new UncheckedIOException(e);
    // }
    // }

    // private final StructPublisher<Pose3d> CamPosePublisher;
    // private final StructPublisher<Transform3d> CamTargetTransformPublisher;

    // private final AprilTagFieldLayout aprilTagFieldLayout;
    // Standard deviations (tune these based on camera characteristics)
    // third parameter should be double the first 2
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
    // the higher the number the less you trust your camera additions
    // private int camProcessorCounter = 0;

    // This is too much voodoo
    // private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    // private final Transform3d floRobotToCam;
    // private final Transform3d fliRobotToCam;
    // private final Transform3d friRobotToCam;
    // private final Transform3d froRobotToCam;

    // private final PhotonCamera floCamera = new PhotonCamera("Front_Left_Outside");
    // private final PhotonCamera fliCamera = new PhotonCamera("Front_Left_Inside");
    // private final PhotonCamera friCamera = new PhotonCamera("Front_Right_Inside");
    // private final PhotonCamera froCamera = new PhotonCamera("Front_Right_Outside");

    // private final PhotonPoseEstimator floEstimator;
    // private final PhotonPoseEstimator fliEstimator;
    // private final PhotonPoseEstimator friEstimator;
    // private final PhotonPoseEstimator froEstimator;
    private final VisionConsumer consumer;

    private final VisionIO[] io;

    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    // Accept a 'mailbox' for us to put `mail` in based on data garnered from the provided cameras
    public Vision(final VisionConsumer consumer, final VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < this.inputs.length; i++) {
            this.inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < this.inputs.length; i++) {
            this.disconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + Integer.toString(i) + " is disconnected.",
                            AlertType.kWarning);
        }

        // // Camera transforms
        // this.floRobotToCam = new Transform3d(
        // new Translation3d(VisionConstants.FLO_X, VisionConstants.FLO_Y,
        // VisionConstants.FLO_Z),
        // new Rotation3d(VisionConstants.FLO_ROLL, VisionConstants.FLO_PITCH,
        // VisionConstants.FLO_YAW));

        // this.fliRobotToCam = new Transform3d(
        // new Translation3d(VisionConstants.FLI_X, VisionConstants.FLI_Y,
        // VisionConstants.FLI_Z),
        // new Rotation3d(VisionConstants.FLI_ROLL, VisionConstants.FLI_PITCH,
        // VisionConstants.FLI_YAW));

        // this.friRobotToCam = new Transform3d(
        // new Translation3d(VisionConstants.FRI_X, VisionConstants.FRI_Y,
        // VisionConstants.FRI_Z),
        // new Rotation3d(VisionConstants.FRI_ROLL, VisionConstants.FRI_PITCH,
        // VisionConstants.FRI_YAW));

        // this.froRobotToCam = new Transform3d(
        // new Translation3d(VisionConstants.FRO_X, VisionConstants.FRO_Y,
        // VisionConstants.FRO_Z),
        // new Rotation3d(VisionConstants.FRO_ROLL, VisionConstants.FRO_PITCH,
        // VisionConstants.FRO_YAW));

        // front camera estimator (new 2026 syntax)
        // this.floEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout,
        // this.floRobotToCam);

        // this.fliEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout,
        // this.fliRobotToCam);

        // this.friEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout,
        // this.friRobotToCam);

        // this.froEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout,
        // this.froRobotToCam);

        // Initialize NetworkTables publishers
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // CamPosePublisher = inst.getStructTopic("/Vision/LeftCameraPose",
        // Pose3d.struct).publish();
        // CamTargetTransformPublisher =
        // inst.getStructTopic("/Vision/LeftCamTargetTransform",
        // Transform3d.struct).publish();

    }

    public boolean isPoseOffField(final Pose3d pose) {
        return !FieldConstants.SAFE_ZONE.contains(new Translation2d(pose.getX(), pose.getY()));
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

    @Override
    public void periodic() {
        // TODO Profile with VisualVM to see what is going on
        for (int i = 0; i < this.io.length; i++) {
            this.io[i].updateInputs(this.inputs[i]);
            // Logger.processInputs("Vision/Camera" + Integer.toString(i), this.inputs[i]);
        }
        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < this.io.length; cameraIndex++) {
            // Update disconnected alert
            this.disconnectedAlerts[cameraIndex].set(!this.inputs[cameraIndex].connected);

            // Initialize logging values
            // final List<Pose3d> tagPoses = new LinkedList<>();
            // final List<Pose3d> robotPoses = new LinkedList<>();
            // final List<Pose3d> robotPosesAccepted = new LinkedList<>();
            // final List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            // for (final int tagId : this.inputs[cameraIndex].tagIds) {
            // final var tagPose = VisionConstants.FIELD_LAYOUT.getTagPose(tagId);
            // if (tagPose.isPresent()) {
            // tagPoses.add(tagPose.get());
            // }
            // }

            // Loop over pose observations
            for (final var observation : this.inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                final var hasTag = observation.tagCount() == 0;
                final var isAmbiguous =
                        observation.ambiguity() > VisionConstants.AMBIGUITY_THRESHOLD;
                final var isPoseOffField = this.isPoseOffField(observation.pose());
                final var isRobotOffTheGround =
                        Math.abs(observation.pose().getZ())
                                > VisionConstants.MAX_Z_HEIGHT.magnitude();
                final boolean rejectPose =
                        !hasTag || !isAmbiguous || isPoseOffField || isRobotOffTheGround;

                // Add pose to log
                // robotPoses.add(observation.pose());
                // if (rejectPose) {
                // robotPosesRejected.add(observation.pose());
                // } else {
                // robotPosesAccepted.add(observation.pose());
                // }

                if (rejectPose) {
                    continue;
                }

                final Matrix<N3, N1> stdDev;
                if (observation.tagCount() == 1) {
                    stdDev = this.singleTagStdDevs;
                } else {
                    stdDev =
                            this.multiTagStdDevs.times(
                                    0.2
                                            + (observation.averageTagDistance()
                                                    * observation.averageTagDistance()
                                                    / 20));
                }
                this.consumer.accept(
                        observation.pose().toPose2d(), observation.timestamp(), stdDev);
            }

            // Log camera metadata
            // Logger.recordOutput(
            // "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            // tagPoses.toArray(new Pose3d[0]));
            // Logger.recordOutput(
            // "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            // robotPoses.toArray(new Pose3d[0]));
            // Logger.recordOutput(
            // "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            // robotPosesAccepted.toArray(new Pose3d[0]));
            // Logger.recordOutput(
            // "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            // robotPosesRejected.toArray(new Pose3d[0]));
            // allTagPoses.addAll(tagPoses);
            // allRobotPoses.addAll(robotPoses);
            // allRobotPosesAccepted.addAll(robotPosesAccepted);
            // allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        // Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        // Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        // Logger.recordOutput(
        // "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        // Logger.recordOutput(
        // "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(
                Pose2d visionRobotPose,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
