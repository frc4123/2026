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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{

    private final Transform3d FLO_robotToCam;
    private final Transform3d FLI_robotToCam;
    private final Transform3d FR_robotToCam;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Standard deviations (tune these based on camera characteristics)
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
    // the higher the number the less you trust your camera additions
    // third parameter should be double the first 2

    // private final StructPublisher<Pose3d> CamPosePublisher;
    // private final StructPublisher<Transform3d> CamTargetTransformPublisher;

    
    private final PhotonCamera FLO_camera = new PhotonCamera("Front_Left_Outside_Arducam_OV9281_USB_Camera");
    private final PhotonCamera FLI_camera = new PhotonCamera("Front_Left_Inside_Arducam_OV9281_USB_Camera");
    private final PhotonCamera FR_camera = new PhotonCamera("Front_Right_Arducam_OV9281_USB_Camera");
    private final PhotonCamera turretCam = new PhotonCamera("Turret_Arducam_OV9281_USB_Camera");

    private final PhotonPoseEstimator FLO_Estimator;
    private final PhotonPoseEstimator FLI_Estimator;
    private final PhotonPoseEstimator FR_Estimator;

    private int camProcessorCounter = 0;

    private static boolean isBlue = false;
    private static boolean isRed = false;

    private final CommandSwerveDrivetrain drivetrain;
    private Oculus oculus;

    public Vision(CommandSwerveDrivetrain drivetrain, Oculus oculus) {
        this.drivetrain = drivetrain;
        this.oculus = oculus;
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

        FR_robotToCam = new Transform3d(
            new Translation3d(
                VisionConstants.FR_frontX,
                VisionConstants.FR_frontY,
                VisionConstants.FR_frontZ),
            new Rotation3d(
                VisionConstants.FR_frontRoll,
                VisionConstants.FR_frontPitch,
                VisionConstants.FR_frontYaw)
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

        FR_Estimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            FR_robotToCam
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

    private void processVision(PhotonCamera camera, PhotonPoseEstimator estimator) {
        
        if(!shouldAcceptPhotonUpdate()) {return;}

        PhotonPipelineResult result = getLatestResults(camera);
        if (result == null) return;
        
        // Get pose using new 2026 methods
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        
        // Strategy 1: Try multi-tag (if available from coprocessor)
        if (result.getMultiTagResult().isPresent()) {
            estimatedPose = estimator.estimateCoprocMultiTagPose(result);
        }
        
        // Strategy 2: Fallback to single tag (lowest ambiguity)
        if (estimatedPose.isEmpty()) {
            estimatedPose = estimator.estimateLowestAmbiguityPose(result);
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
    
    private boolean shouldAcceptPhotonUpdate() {
        // Check 1: Robot pitch/roll (are we tilted like going over bump?)
        Rotation3d rotation = drivetrain.getRotation3d();
        double pitchDeg = Math.toDegrees(rotation.getY());
        double rollDeg = Math.toDegrees(rotation.getX());
        
        if (Math.abs(pitchDeg) > VisionConstants.MAX_ACCEPTABLE_PITCH || 
            Math.abs(rollDeg) > VisionConstants.MAX_ACCEPTABLE_PITCH) {
            return false; // Robot is tilted - probably on bump or climb
        }
        
        // Check 2: Z-axis acceleration (are we bouncing/airborne?)
        // If you have an IMU with 3-axis accel:
        // double zAccel = navX.getRawAccelZ(); // or pigeon.getAccelZ()
        // if (Math.abs(zAccel - 9.81) > MAX_ACCEPTABLE_Z_ACCEL) {
        //     return false; // Experiencing high vertical acceleration
        // }
        
        // Check 3: Large pose jumps (vision suddenly disagrees with odometry)
        // Add standard deviation checking here if needed
        
        return true; // Accept the update
    }

    public double getTurretCamOffset() {
        PhotonPipelineResult result = getLatestResults(turretCam);

        if (result == null || result.getTargets().isEmpty()) {
            return 0.0; // no targets
        }

        for (var target : result.getTargets()) {
            int id = target.getFiducialId();

            boolean validTag = false;

            if (isBlue) {
                for (double validId : TurretConstants.validTurretTagsBlue) {
                    if (id == (int) validId) {
                        validTag = true;
                        break;
                    }
                }
            } else if (isRed) {
                for (double validId : TurretConstants.validTurretTagsRed) {
                    if (id == (int) validId) {
                        validTag = true;
                        break;
                    }
                }
            }

            if (!validTag) continue;

            // Get camera yaw (degrees) to this tag
            double photonYaw = target.getYaw();

            // XY distance from camera to tag
            double dx = target.getBestCameraToTarget().getX(); // forward
            double dy = target.getBestCameraToTarget().getY(); // sideways

            double[] offsets = getHubOffsetForTag(target.getFiducialId()); // returns {offsetX, offsetY}
            dx -= offsets[0]; // forward adjustment
            dy -= offsets[1]; // sideways adjustment

            // Compute turret angle to hub center without X/Y offset yet
            // Hub offset will be applied in a separate method
            double angleToHubCenterRad = Math.atan2(dy, dx); // relative to camera forward
            double angleToHubCenterDeg = Math.toDegrees(angleToHubCenterRad);

            // Combine camera yaw with raw geometry
            double turretAngle = photonYaw + angleToHubCenterDeg;

            return turretAngle;
        }

        return 0.0; // no valid target found
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

    public Translation2d getHub() {
        if(isBlue == false && isRed == false){
            if(DriverStation.isDSAttached()){
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
                isRed = DriverStation.getAlliance().get() == Alliance.Red ? true : false;
            } else {
                isBlue = false;
                isRed = false;
            }
        }

        Translation2d blueHub = VisionConstants.blueHubTranslation2d;
        Translation2d redHub = VisionConstants.redHubTranslation2d;

        if(isRed){
            return redHub;

        } else {return blueHub;}
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

    public boolean isBlue(){
        if(isBlue == false && isRed == false){
            if(DriverStation.isDSAttached()){
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
                isRed = DriverStation.getAlliance().get() == Alliance.Red ? true : false;
            } else {
                isBlue = false;
                isRed = false;
            }
        }
        return isBlue;
    }

    public boolean isRed(){
        if(isBlue == false && isRed == false){
            if(DriverStation.isDSAttached()){
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
                isRed = DriverStation.getAlliance().get() == Alliance.Red ? true : false;
            } else {
                isBlue = false;
                isRed = false;
            }
        }
        return isRed;
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

        double x = robotPose.getX();
        double y = robotPose.getY();

        if (isBlue) {

            boolean isBumpOrTrench = (
                x >= VisionConstants.blueLeftBumpOrTrenchThreshold && 
                x <= VisionConstants.blueRightBumpOrTrenchThreshold
            ) ? true : false;

            if (isBumpOrTrench){
                return getBumpOrTrench(y, robotPose.getRotation());
            } else if(x < VisionConstants.blueHub.getX()){
                return getAngleToTarget(robotPose, VisionConstants.blueHub.getTranslation().toTranslation2d());
                // Check Y zones from top to bottom
            } else if (y >= 5.029) {
                // Top zone - face depot
                return getAngleToTarget(robotPose, VisionConstants.blueDepot.getTranslation().toTranslation2d());
            } else if (y > 4.044) {
                // Upper middle zone - face left bump corner
                return getAngleToTarget(robotPose, VisionConstants.blueLeftBumpCorner.getTranslation().toTranslation2d());
            } else if (y > 3.059) {
                // Lower middle zone - face right bump corner
                return getAngleToTarget(robotPose, VisionConstants.blueRightBumpCorner.getTranslation().toTranslation2d());
            } else {
                // Bottom zone - face aim threshold
                return getAngleToTarget(robotPose, VisionConstants.blueAimThreshold.getTranslation().toTranslation2d());
            }

        } else if (isRed) {
            // Check Y zones from top to bottom
            boolean isBumpOrTrench = (
                x >= VisionConstants.redLeftBumpOrTrenchThreshold && 
                x <= VisionConstants.redRightBumpOrTrenchThreshold
            ) ? true : false;

            if (isBumpOrTrench){
                return getBumpOrTrench(y, robotPose.getRotation());
            } else if(x > VisionConstants.redHub.getX()){
                return getAngleToTarget(robotPose, VisionConstants.redHub.getTranslation().toTranslation2d());
            // Check Y zones from top to bottom
            } else if (y >= 5.029) {
                // Top zone - face aim threshold
                return getAngleToTarget(robotPose, VisionConstants.redAimThreshold.getTranslation().toTranslation2d());
            } else if (y > 4.044) {
                // Upper middle zone - face right bump corner
                return getAngleToTarget(robotPose, VisionConstants.redRightBumpCorner.getTranslation().toTranslation2d());
            } else if (y > 3.059) {
                // Lower middle zone - face left bump corner
                return getAngleToTarget(robotPose, VisionConstants.redLeftBumpCorner.getTranslation().toTranslation2d());
            } else {
                // Bottom zone - face depot
                return getAngleToTarget(robotPose, VisionConstants.redDepot.getTranslation().toTranslation2d());
            }
        }
        
        return new Rotation2d(0);
    }

    private Rotation2d getBumpOrTrench(double y, Rotation2d rotation) {
        if (y >= VisionConstants.topBumpTrenchEdge ||
            y <= VisionConstants.bottomBumpTrenchEdge) {
            
            // Top or bottom - use 0 or 180 based on which is closer
            double deg = rotation.getDegrees();
            double closest = (Math.abs(deg) < 90 || Math.abs(deg) > 270) ? 0 : 180;
            return new Rotation2d(Math.toRadians(closest));
            
        } else {
            
            // Middle trench - use 45 or 225 based on which is closer
            double deg = rotation.getDegrees();
            // Normalize to 0-360
            while (deg < 0) deg += 360;
            while (deg >= 360) deg -= 360;
            
            double closest = (deg < 135 || deg >= 315) ? 45 : 225;
            return new Rotation2d(Math.toRadians(closest));
        }
    }

    // Helper method to calculate angle
    private Rotation2d getAngleToTarget(Pose2d robotPose, Translation2d target) {
        Translation2d delta = target.minus(robotPose.getTranslation());
        return delta.getAngle();
    }

    public double targetFF(Pose2d robotPose, Translation2d targetPosition, ChassisSpeeds robotVelocity) {
        // Vector from robot to target
        Translation2d toTarget = targetPosition.minus(robotPose.getTranslation());
    
        // Distance squared to target
        double distanceSquared = toTarget.getX() * toTarget.getX() + toTarget.getY() * toTarget.getY();
    
        // Avoid division by zero when very close to target
        if (distanceSquared < 0.0001) {
            return 0.0;
        }
    
        // Cross product of velocity and position vector
        // ω = (v × r) / |r|²
        double crossProduct = robotVelocity.vxMetersPerSecond * toTarget.getY() - robotVelocity.vyMetersPerSecond * toTarget.getX();
    
        return crossProduct / distanceSquared;
    }

    @Override
    public void periodic() {
        switch(camProcessorCounter % 3) {
            case 0: processVision(FLO_camera, FLO_Estimator); break;
            case 1: processVision(FLI_camera, FLI_Estimator); break;
            case 2: processVision(FR_camera, FR_Estimator); break;
        }
        camProcessorCounter++;
    }

}
