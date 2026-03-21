package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

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
import org.photonvision.PhotonUtils;
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
import frc.robot.utils.Field;

public class Vision extends SubsystemBase{

    private final Transform3d FLO_robotToCam;
    private final Transform3d FLI_robotToCam;
    private final Transform3d FRI_robotToCam;
    private final Transform3d FRO_robotToCam;

    private final AprilTagFieldLayout aprilTagFieldLayout;

    // Standard deviations (tune these based on camera characteristics)
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.4);
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
    private static double maxDistance = 3.0;

    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private Oculus oculus;

    public Vision(Oculus oculus) {
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
        
        if(!shouldAcceptPhotonUpdate()) {return;}

        PhotonPipelineResult result = getLatestResults(camera);
        if (result == null) return;
        
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        
        // Strategy 1: Try multi-tag (if available from coprocessor)
        if (result.getMultiTagResult().isPresent()) {
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getPoseAmbiguity() > VisionConstants.ambiguityThreshold) {
                    return; //reject measurement
                }
            }
            estimatedPose = estimator.estimateCoprocMultiTagPose(result);
        }
    
        if (estimatedPose.isPresent()) {
            EstimatedRobotPose est = estimatedPose.get();
            Matrix<N3, N1> stdDevs = calculateStdDevs(est, result.getTargets());
            
            if(result.getBestTarget().getPoseAmbiguity() < VisionConstants.ambiguityThreshold){
                for (PhotonTrackedTarget target : result.getTargets()) {
                    double distance = PhotonUtils.calculateDistanceToTargetMeters(
                        estimator.getRobotToCameraTransform().getZ(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ(),
                        estimator.getRobotToCameraTransform().getRotation().getMeasureY().in(Degrees),
                        target.getPitch()
                    );

                    if (distance > maxDistance) {
                        return;
                    }
                }
                    swerve.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds,
                        stdDevs
                    );

                // Reset QuestNav when we have confident AprilTag measurement
                if (shouldResetQuestNav(result) && oculus.isQuestNavConnected()) {
                    oculus.setRobotPose(est.estimatedPose);
                }
            }
        }
    }
    
    private boolean shouldAcceptPhotonUpdate() {
        return true;
        // // Check 1: Robot pitch/roll (are we tilted like going over bump?)
        // Rotation3d rotation = swerve.getRotation3d();
        // double pitchDeg = Math.toDegrees(rotation.getY());
        // double rollDeg = Math.toDegrees(rotation.getX());
        
        // if (Math.abs(pitchDeg) > VisionConstants.MAX_ACCEPTABLE_PITCH || 
        //     Math.abs(rollDeg) > VisionConstants.MAX_ACCEPTABLE_PITCH) {
        //     return false; // Robot is tilted - probably on bump or climb
        // }
        
        // // Check 2: Z-axis acceleration (are we bouncing/airborne?)
        // // If you have an IMU with 3-axis accel:
        // // double zAccel = navX.getRawAccelZ(); // or pigeon.getAccelZ()
        // // if (Math.abs(zAccel - 9.81) > MAX_ACCEPTABLE_Z_ACCEL) {
        // //     return false; // Experiencing high vertical acceleration
        // // }
        
        // // Check 3: Large pose jumps (vision suddenly disagrees with odometry)
        // // Add standard deviation checking here if needed
        
        // return true; // Accept the update
    }

    // public double getTurretCamOffset() {
    //     PhotonPipelineResult result = getLatestResults(turretCam);

    //     if (result == null || result.getTargets().isEmpty()) {
    //         return 0.0; // no targets
    //     }

    //     for (var target : result.getTargets()) {
    //         int id = target.getFiducialId();

    //         boolean validTag = false;

    //         if (isBlue) {
    //             for (double validId : TurretConstants.validTurretTagsBlue) {
    //                 if (id == (int) validId) {
    //                     validTag = true;
    //                     break;
    //                 }
    //             }
    //         } else if (isRed) {
    //             for (double validId : TurretConstants.validTurretTagsRed) {
    //                 if (id == (int) validId) {
    //                     validTag = true;
    //                     break;
    //                 }
    //             }
    //         }

    //         if (!validTag) continue;

    //         // Get camera yaw (degrees) to this tag
    //         double photonYaw = target.getYaw();

    //         // XY distance from camera to tag
    //         double dx = target.getBestCameraToTarget().getX(); // forward
    //         double dy = target.getBestCameraToTarget().getY(); // sideways

    //         double[] offsets = getHubOffsetForTag(target.getFiducialId()); // returns {offsetX, offsetY}
    //         dx -= offsets[0]; // forward adjustment
    //         dy -= offsets[1]; // sideways adjustment

    //         // Compute turret angle to hub center without X/Y offset yet
    //         // Hub offset will be applied in a separate method
    //         double angleToHubCenterRad = Math.atan2(dy, dx); // relative to camera forward
    //         double angleToHubCenterDeg = Math.toDegrees(angleToHubCenterRad);

    //         // Combine camera yaw with raw geometry
    //         double turretAngle = photonYaw + angleToHubCenterDeg;

    //         return turretAngle;
    //     }

    //     return 0.0; // no valid target found
    // }

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
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getPoseAmbiguity() > VisionConstants.ambiguityThreshold) {
                    return false; //reject measurement
                }
            }
            return true; // Multi-tag = high confidence
        }
        
        if (result.getBestTarget() != null) {
            // Single tag with low ambiguity and close distance
            return result.getBestTarget().getPoseAmbiguity() < VisionConstants.ambiguityThreshold;
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
        switch(camProcessorCounter % 4) {
            case 0: processVision(FLO_camera, FLO_Estimator); break;
            case 1: processVision(FLI_camera, FLI_Estimator); break;
            case 2: processVision(FRI_camera, FRI_Estimator); break;
            case 3: processVision(FRO_camera, FRO_Estimator); break;
        }
        camProcessorCounter++;
    }

}
