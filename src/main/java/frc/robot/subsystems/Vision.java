package frc.robot.subsystems;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase{

    private final Transform3d robotToCam;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Standard deviations (tune these based on camera characteristics)
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
    // the higher the number the less you trust your camera additions
    // third parameter should be double the first 2

    // private final StructPublisher<Pose3d> CamPosePublisher;
    // private final StructPublisher<Transform3d> CamTargetTransformPublisher;

    private final PhotonPoseEstimator frontEstimator;
    private final PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera");

    private static boolean isBlue = false;
    private static boolean isRed = false;

    private final CommandSwerveDrivetrain drivetrain;
    private Oculus oculus;

    public Vision(CommandSwerveDrivetrain drivetrain, Oculus oculus) {
        this.drivetrain = drivetrain;
        this.oculus = oculus;
        this.aprilTagFieldLayout = loadAprilTagFieldLayout("/fields/2026Welded.json");

        // Camera transforms
        robotToCam = new Transform3d(
            new Translation3d(
                Constants.VisionConstants.frontX,
                Constants.VisionConstants.frontY,
                Constants.VisionConstants.frontZ),
            new Rotation3d(
               Constants.VisionConstants.frontRoll,
                Constants.VisionConstants.frontPitch,
                Constants.VisionConstants.frontYaw)
        );

        // front camera estimator (new 2026 syntax)
        frontEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            robotToCam
        );

        // Initialize NetworkTables publishers
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // CamPosePublisher = inst.getStructTopic("/Vision/LeftCameraPose", Pose3d.struct).publish();
        // CamTargetTransformPublisher = inst.getStructTopic("/Vision/LeftCamTargetTransform", Transform3d.struct).publish();
        
    }

    public static PhotonPipelineResult getLatestResults(PhotonCamera camera) {
        List<PhotonPipelineResult> currentResultList = camera.getAllUnreadResults();
        
        // Search from newest (end of list) to oldest (start of list)
        for (int i = currentResultList.size() - 1; i >= 0; i--) {
            PhotonPipelineResult result = currentResultList.get(i);
            if (result.hasTargets()) {
                return result;  // Return first valid result with targets
            }
        }
        return null;  // No valid results found
    }

    private void processVision(PhotonCamera camera) {
        PhotonPipelineResult result = getLatestResults(camera);
        if (result == null) return;
        
        // Get pose using new 2026 methods
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        
        // Strategy 1: Try multi-tag (if available from coprocessor)
        if (result.getMultiTagResult().isPresent()) {
            estimatedPose = frontEstimator.estimateCoprocMultiTagPose(result);
        }
        
        // Strategy 2: Fallback to single tag (lowest ambiguity)
        if (estimatedPose.isEmpty()) {
            estimatedPose = frontEstimator.estimateLowestAmbiguityPose(result);
        }
        
        if (estimatedPose.isPresent()) {
            EstimatedRobotPose est = estimatedPose.get();
            Matrix<N3, N1> stdDevs = calculateStdDevs(est, result.getTargets());
            
            // Add vision measurement to pose estimator
            if(result.getBestTarget().getPoseAmbiguity() < 0.1){
                drivetrain.addVisionMeasurement(
                    est.estimatedPose.toPose2d(),
                    est.timestampSeconds,
                    stdDevs
                );
            }
            
            // Reset QuestNav when we have confident AprilTag measurement
            if (shouldResetQuestNav(result) && oculus.isQuestNavConnected()) {
                oculus.setRobotPose(est.estimatedPose);
            }
        }
    }
        
    private boolean shouldResetQuestNav(PhotonPipelineResult result) {
        // Only reset QuestNav with high-confidence measurements
        if (result.getMultiTagResult().isPresent()) {
            return true; // Multi-tag = high confidence
        }
        
        if (result.getBestTarget() != null) {
            // Single tag with low ambiguity and close distance
            return result.getBestTarget().getPoseAmbiguity() < 0.1;
        }
        
        return false;
    }

    // private void publishTargetTransform(PhotonTrackedTarget target, boolean isRightCamera) {
    //     Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    //     if (tagPose.isEmpty()) return;

    //     Transform3d cameraToTarget = target.getBestCameraToTarget();
    //     Transform3d robotToTarget = robotToCam.plus(cameraToTarget);
    //     CamTargetTransformPublisher.set(robotToTarget);
    // }

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
            
            numTags++;
            totalDistance += tagPose.get().toPose2d().getTranslation()
                .getDistance(est.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) return singleTagStdDevs;
        
        double avgDistance = totalDistance / numTags;
        Matrix<N3, N1> baseDevs = numTags >= 2 ? multiTagStdDevs : singleTagStdDevs;
        return baseDevs.times(0.2 + (avgDistance * avgDistance / 20));
    }

    public Rotation2d angleToFace(Pose2d robotPose) {
        if(isBlue == false && isRed == false){
            if(DriverStation.isDSAttached()){
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
                isRed = DriverStation.getAlliance().get() == Alliance.Red ? true : false;
            } else {
                isBlue = false;
                isRed = false;
            }
        }

        Translation2d target = null;
        Pose3d blueHub = Constants.VisionConstants.blueHub;
        Pose3d redHub = Constants.VisionConstants.redHub;

        if(isBlue && robotPose.getX() < blueHub.getX()){
            target = Constants.VisionConstants.blueHub.getTranslation().toTranslation2d();
            Translation2d robotPos = robotPose.getTranslation();
            Translation2d delta = target.minus(robotPos);
            return delta.getAngle();
        } else if (isRed && robotPose.getX() > redHub.getX()){
            target = Constants.VisionConstants.redHub.getTranslation().toTranslation2d();
            Translation2d robotPos = robotPose.getTranslation();
            Translation2d delta = target.minus(robotPos);
            return delta.getAngle();
        }
        /*this should allow the robot to face the hub from whatever position it is
        we will use this command if our turret breaks and we havfe to start auto aiming using swerve and not turret
        */

        return new Rotation2d(0);
        /*
        if it does this then this function didnt work :(, this line only exists so that we dont crash code 
        we have to return something since the functino isnt void
        */

        /* we can also reuse ts for turret because this function is just telling us what angle to face our scoring point
        based on position */
    }

    @Override
    public void periodic() {
        processVision(camera);
    }

}
