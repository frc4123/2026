package frc.robot.subsystems;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

import frc.robot.utils.Field;

public class Vision extends SubsystemBase {

    private final Transform3d FLO_robotToCam;
    private final Transform3d FLI_robotToCam;
    private final Transform3d FRI_robotToCam;
    private final Transform3d FRO_robotToCam;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Standard deviations (tune these based on camera characteristics)
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    // the higher the number the less you trust your camera additions
    // third parameter should be double the first 2

    // private final StructPublisher<Pose3d> CamPosePublisher;
    // private final StructPublisher<Transform3d> CamTargetTransformPublisher;

    
    private final PhotonCamera FLO_camera = new PhotonCamera("Front_Left_Outside");
    private final PhotonCamera FLI_camera = new PhotonCamera("Front_Left_Inside");
    private final PhotonCamera FRI_camera = new PhotonCamera("Front_Right_Inside");
    private final PhotonCamera FRO_camera = new PhotonCamera("Front_Right_Outside");

    private final PhotonPoseEstimator FLO_Estimator;
    private final PhotonPoseEstimator FLI_Estimator;
    private final PhotonPoseEstimator FRI_Estimator;
    private final PhotonPoseEstimator FRO_Estimator;

    private int camProcessorCounter = 0;

    private static boolean isBlue = false;
    private static boolean isRed = false;

    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    public Vision(Oculus oculus) {
        this.aprilTagFieldLayout = loadAprilTagFieldLayout("/fields/2026Welded.json");

        // Camera transforms
        FLO_robotToCam = new Transform3d(
            new Translation3d(
                VisionConstants.FLO_frontX,
                VisionConstants.FLO_frontY,
                VisionConstants.FLO_frontZ),
            new Rotation3d(
                VisionConstants.FLO_frontRoll,
                VisionConstants.FLO_frontPitch,
                VisionConstants.FLO_frontYaw)
        );

        FLI_robotToCam = new Transform3d(
            new Translation3d(
                VisionConstants.FLI_frontX,
                VisionConstants.FLI_frontY,
                VisionConstants.FLI_frontZ),
            new Rotation3d(
                VisionConstants.FLI_frontRoll,
                VisionConstants.FLI_frontPitch,
                VisionConstants.FLI_frontYaw)
        );

        FRI_robotToCam = new Transform3d(
            new Translation3d(
                VisionConstants.FRI_frontX,
                VisionConstants.FRI_frontY,
                VisionConstants.FRI_frontZ),
            new Rotation3d(
                VisionConstants.FRI_frontRoll,
                VisionConstants.FRI_frontPitch,
                VisionConstants.FRI_frontYaw)
        );

        FRO_robotToCam = new Transform3d(
            new Translation3d(
                VisionConstants.FRO_frontX,
                VisionConstants.FRO_frontY,
                VisionConstants.FRO_frontZ),
            new Rotation3d(
                VisionConstants.FRO_frontRoll,
                VisionConstants.FRO_frontPitch,
                VisionConstants.FRO_frontYaw)
        );

        // front camera estimator (new 2026 syntax)
        FLO_Estimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            FLO_robotToCam
        );

        FLI_Estimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            FLI_robotToCam
        );

        FRI_Estimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            FRI_robotToCam
        );

        FRO_Estimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            FRO_robotToCam
        );

        // Initialize NetworkTables publishers
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // CamPosePublisher = inst.getStructTopic("/Vision/LeftCameraPose", Pose3d.struct).publish();
        // CamTargetTransformPublisher = inst.getStructTopic("/Vision/LeftCamTargetTransform", Transform3d.struct).publish();
        
    }


    public static PhotonPipelineResult getLatestResults(PhotonCamera camera) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return null;
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);

        if (!latest.hasTargets()) {
            return null;
        }

        return latest;
    }

    private void processVision(PhotonCamera camera, PhotonPoseEstimator estimator) {

        PhotonPipelineResult result = getLatestResults(camera);
        if (result == null) return;
        
        List<PhotonTrackedTarget> validTargets = getValidTargets(result, estimator);

        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        PhotonPipelineResult filteredResult = new PhotonPipelineResult(
            result.metadata,
            validTargets,
            Optional.empty()
        );

        if (validTargets.size() > 1) {
        
            estimatedPose = estimator.estimateCoprocMultiTagPose(filteredResult);
            
        } else {
            Optional<EstimatedRobotPose> singleTagPose =
                estimator.estimateLowestAmbiguityPose(filteredResult);

            if (singleTagPose.isPresent()) {
                PhotonTrackedTarget best = result.getBestTarget();
                if (best != null &&
                    best.getPoseAmbiguity() < VisionConstants.ambiguityThreshold) {
                    estimatedPose = singleTagPose;
                }
            }
        }

        if (estimatedPose.isPresent()) {
            EstimatedRobotPose est = estimatedPose.get();

            Matrix<N3, N1> stdDevs = calculateStdDevs(est, validTargets);

            swerve.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                stdDevs
            );
        }
    }

    private List<PhotonTrackedTarget> getValidTargets(PhotonPipelineResult result, PhotonPoseEstimator estimator) {
        List<PhotonTrackedTarget> validTargets = new ArrayList<PhotonTrackedTarget>();
        for (PhotonTrackedTarget target : result.getTargets()) {

            if (target.getPoseAmbiguity() > VisionConstants.ambiguityThreshold) {
                continue;
            }
            
            validTargets.add(target);
        }

        return validTargets;
    }
    
    public double[] getHubOffsetForTag(int id){
        switch(id){
            case 21: return new double[]{0.0, TurretConstants.tagOffset};
            case 26: return new double[]{-TurretConstants.tagOffset, 0.0};
            case 18: return new double[]{0.0, -TurretConstants.tagOffset};

            case 2: return new double[]{0.0, TurretConstants.tagOffset};
            case 10: return new double[]{TurretConstants.tagOffset, 0.0};
            case 5: return new double[]{0.0, -TurretConstants.tagOffset};
        }
        return new double[] {0.0,0.0};
    }

    public static AprilTagFieldLayout loadAprilTagFieldLayout(String resourceFile) {
        try (InputStream is = Vision.class.getResourceAsStream(resourceFile);
             InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8)) {
            return new ObjectMapper().readValue(isr, AprilTagFieldLayout.class);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est, List<PhotonTrackedTarget> targets) {
        int numTags = 0;
        double totalDistance = 0;

        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;

            // Reject low-ambiguity threshold — tune this
            if (target.getPoseAmbiguity() > VisionConstants.ambiguityThreshold) continue;

            numTags++;
            totalDistance += tagPose.get().toPose2d().getTranslation()
                .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        double avgDistance = totalDistance / numTags;

        // Hard reject if too far — tags beyond 4m are unreliable
        if (avgDistance > 4.0) return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

        Matrix<N3, N1> baseDevs = singleTagStdDevs;

        // Aggressive cubic scaling instead of quadratic
        double scalar = 1 + (avgDistance * avgDistance * avgDistance / 15.0);

        //extra penalty
        scalar *= 1.7; //2.0 3.0

        return baseDevs.times(scalar);
    }

    public Pose3d getHub3D() {
        if(isBlue == false && isRed == false){
            if(DriverStation.isDSAttached()){
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
                isRed = DriverStation.getAlliance().get() == Alliance.Red ? true : false;
            } else {
                isBlue = false;
                isRed = false;
            }
        }

        Pose3d blueHub = VisionConstants.blueHub;
        Pose3d redHub = VisionConstants.redHub;

        if(isRed){
            return redHub;

        } else {return blueHub;}
    }
    public Rotation2d getTrenchAngle(double x) {
        if (Field.isBlue()) {
            if (x > 4.63){
                return new Rotation2d(Math.toRadians(0));
            } else {
                return new Rotation2d(Math.toRadians(180));
            }
        } else { 
            if (x > 11.91){
                return new Rotation2d(Math.toRadians(180));
            } else {
                return new Rotation2d(Math.toRadians(0));
            }
        }
    }

    public boolean isTagHub(PhotonPipelineResult result){
        if(result == null) {return false;}
        int tagId = result.getBestTarget().getFiducialId();
        if(Set.of(18,19,20,21,24,25,26,27).contains(tagId) && Field.isBlue()){
            return true;
        } else if (Set.of(8,9,10,11,2,3,4,5).contains(tagId) && Field.isRed()) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        switch(camProcessorCounter % 4) {
            case 0: processVision(FLO_camera, FLO_Estimator); break;
            case 1: processVision(FLI_camera, FLI_Estimator); break;
            case 2: processVision(FRI_camera, FRI_Estimator); break;
            case 3: processVision(FRO_camera, FRO_Estimator); break;
        }
        camProcessorCounter++;
    }
}