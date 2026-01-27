package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Turret subsystem for field-relative aiming.
 * Tracks cumulative angle using CANCoder with unwrapping.
 * Physical range ±360 degrees from turret zero.
 * Uses velocity feedforward to cancel robot yaw motion.
 */
public class Turret extends SubsystemBase {

    CANBus canivore = new CANBus("Canivore");
    
    // Hardware
    private final TalonFX turretMotor = new TalonFX(Constants.CanIdCanivore.Turret, canivore);
    private final CANcoder turretEncoder = new CANcoder(Constants.CanIdCanivore.Turret_Encoder);

    // Alliance tracking
    private static boolean isBlue = false;
    private static boolean isRed = false;

    // Motion Magic controller
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic =
            new DynamicMotionMagicTorqueCurrentFOC(
                    Constants.Turret.stowPosition,
                    Constants.Turret.velocity,
                    Constants.Turret.acceleration
            );

    // Gear ratio turret motor -> turret shaft
    private final double gearRatio = Constants.Turret.gearRatio;

    // Physical turret limits relative to turret zero
    private final double minCumulativeAngle = -360.0;
    private final double maxCumulativeAngle = 360.0;

    // Cumulative turret angle tracking (robot-relative)
    private double cumulativeAngle;
    private double prevAbsolute;
    
    // Simulation-specific tracking
    private double simulatedFieldAngle = 0.0;  // Track field-relative angle in sim

    // Swerve and vision references
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    public Turret(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        configureMotor();
        configureEncoder();

        // Read encoder once at startup
        double initial = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        cumulativeAngle = initial;
        prevAbsolute = initial;
    }

    private void configureMotor() {
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
                .withKP(Constants.Turret.kP)
                .withKI(Constants.Turret.kI)
                .withKD(Constants.Turret.kD)
                .withKV(Constants.Turret.kV)
                .withKA(Constants.Turret.kA);

        turretMotor.getConfigurator().apply(pid);
    }

    private void configureEncoder() {
        // Encoder configuration if needed
    }

    private double degreesToMotorRotations(double deg) {
        return deg / 360.0 * gearRatio;
    }

    private double normalizeAngle(double deg) {
        deg %= 360.0;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

    /**
     * Unwrap absolute encoder into cumulative turret angle.
     * Call exactly once per loop.
     */
    private void updateCumulativeAngle() {
        double abs = turretEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        double delta = abs - prevAbsolute;

        // Handle wraparound
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        cumulativeAngle += delta;
        prevAbsolute = abs;
    }

    /**
     * Simulates the encoder reading for simulation mode.
     * Updates cumulativeAngle based on simulated turret movement.
     */
    private void updateSimulatedEncoder() {
        // Get target field angle
        Rotation2d targetFieldAngle = targetAngle(drivetrain.getState().Pose);
        double targetFieldDeg = targetFieldAngle.getDegrees();
        
        // Simulate turret rotation toward target (in field frame)
        double step = 25.0;  // max degrees per loop
        double diff = normalizeAngle(targetFieldDeg - simulatedFieldAngle);
        
        if (Math.abs(diff) > step) {
            simulatedFieldAngle += Math.copySign(step, diff);
        } else {
            simulatedFieldAngle = targetFieldDeg;
        }
        
        // Convert field angle to what the encoder would read (robot-relative)
        double robotHeading = drivetrain.getState().Pose.getRotation().getDegrees();
        double encoderReading = normalizeAngle(simulatedFieldAngle - robotHeading);
        
        // Normalize to [0, 360) like a real absolute encoder
        if (encoderReading < 0) encoderReading += 360.0;
        
        // Update cumulative from encoder delta
        double delta = encoderReading - prevAbsolute;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        
        cumulativeAngle += delta;
        prevAbsolute = encoderReading;
    }

    /**
     * Calculate the field-relative angle the turret should point to hit the target.
     * Returns angle to blue or red hub based on alliance and robot position.
     */
    public Rotation2d targetAngle(Pose2d robotPose) {
        Translation2d target;
        Pose3d blueHub = Constants.VisionConstants.blueHub;
        Pose3d redHub = Constants.VisionConstants.redHub;

        if (isBlue && robotPose.getX() < blueHub.getX()) {
            target = Constants.VisionConstants.blueHub.getTranslation().toTranslation2d();
        } else if (isRed && robotPose.getX() > redHub.getX()) {
            target = Constants.VisionConstants.redHub.getTranslation().toTranslation2d();
        } else {
            // No valid target - return 0 to avoid crashes
            return new Rotation2d(0);
        }
        
        // Calculate turret position accounting for offset from robot center
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d turretPos = robotPos.plus(
            Constants.Turret.turretOffset.rotateBy(robotPose.getRotation())
        );

        // Calculate angle from turret to target
        Translation2d delta = target.minus(turretPos);
        return delta.getAngle();
    }

    /**
     * Calculate feedforward for robot translation causing angle to target to change.
     * Compensates for the turret needing to track the target as the robot drives.
     */
    private double calculateTranslationFeedforward() {
        Pose2d robotPose = drivetrain.getState().Pose;
        
        // Same target selection logic as targetAngle
        Translation2d target;
        Pose3d blueHub = Constants.VisionConstants.blueHub;
        Pose3d redHub = Constants.VisionConstants.redHub;

        if (isBlue && robotPose.getX() < blueHub.getX()) {
            target = Constants.VisionConstants.blueHub.getTranslation().toTranslation2d();
        } else if (isRed && robotPose.getX() > redHub.getX()) {
            target = Constants.VisionConstants.redHub.getTranslation().toTranslation2d();
        } else {
            return 0.0;
        }
        
        // Account for turret offset from robot center
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d turretPos = robotPos.plus(
            Constants.Turret.turretOffset.rotateBy(robotPose.getRotation())
        );
                
        // Vector from turret to target
        Translation2d toTarget = target.minus(turretPos);
        double distanceSquared = toTarget.getNorm() * toTarget.getNorm();
        
        if (distanceSquared < 0.0001) {
            return 0.0; // Avoid division by zero
        }
        
        // Get robot velocity in field frame
        var robotSpeeds = drivetrain.getState().Speeds;
        
        // Cross product gives angular velocity (rad/s)
        double crossProduct = robotSpeeds.vxMetersPerSecond * toTarget.getY() - 
                            robotSpeeds.vyMetersPerSecond * toTarget.getX();
        
        double angularVelocity_radPerSec = crossProduct / distanceSquared;
        
        // Convert to degrees per second
        return angularVelocity_radPerSec * 180.0 / Math.PI;
    }

    /**
     * Field-relative turret control with velocity feedforward.
     * Compensates for both robot rotation and translation.
     */
    public void setFieldAngle(Rotation2d targetFieldAngle, double cameraOffset) {
        // Clamp vision offset
        cameraOffset = Math.max(-1.0, Math.min(1.0, cameraOffset));

        // Robot heading and yaw rate
        Rotation2d robotHeading = drivetrain.getState().Pose.getRotation();
        double robotYawRateDegPerSec = drivetrain.getState().Speeds.omegaRadiansPerSecond * 180.0 / Math.PI;

        // Convert field target into robot-relative turret target
        double targetTurretAngle = normalizeAngle(targetFieldAngle.minus(robotHeading).getDegrees());

        // Compute shortest delta to target
        double delta = normalizeAngle(targetTurretAngle - cumulativeAngle);

        // Compute new cumulative setpoint
        double targetCumulative = cumulativeAngle + delta + cameraOffset;

        // Clamp to physical limits with wrapping
        if (targetCumulative > maxCumulativeAngle) {
            targetCumulative -= 360.0;
        } else if (targetCumulative < minCumulativeAngle) {
            targetCumulative += 360.0;
        }
        targetCumulative = Math.max(minCumulativeAngle, Math.min(maxCumulativeAngle, targetCumulative));

        // ========== FEEDFORWARD CALCULATION ==========
        
        // 1. Robot rotation feedforward (compensates for robot spinning)
        double rotationFF_degPerSec = -robotYawRateDegPerSec;
        
        // 2. Robot translation feedforward (compensates for robot driving)
        double translationFF_degPerSec = calculateTranslationFeedforward();
        
        // 3. Total feedforward
        double totalFF_degPerSec = rotationFF_degPerSec + translationFF_degPerSec;
        double totalFF_rotPerSec = totalFF_degPerSec / 360.0 * gearRatio;

        // Convert position target to motor rotations
        double targetRotations = degreesToMotorRotations(targetCumulative);

        // Command Motion Magic with combined velocity feedforward
        turretMotor.setControl(
                motionMagic
                        .withPosition(targetRotations)
                        .withFeedForward(totalFF_rotPerSec)
        );
    }

    /**
     * Relative manual rotation command.
     */
    public void rotateRelative(double deltaDeg) {
        double target = cumulativeAngle + deltaDeg;
        target = Math.max(minCumulativeAngle, Math.min(maxCumulativeAngle, target));

        double targetRotations = degreesToMotorRotations(target);
        turretMotor.setControl(motionMagic.withPosition(targetRotations));
    }

    /**
     * Get the cumulative robot-relative turret angle (can exceed ±360°).
     */
    public double getCumulativeAngle() {
        return cumulativeAngle;
    }

    /**
     * Get the raw absolute encoder position.
     */
    public double getAbsoluteAngle() {
        return turretEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Get the field-relative turret angle.
     * Converts robot-relative cumulative angle to field frame.
     */
    public double getFieldAngle() {
        double robotHeading = drivetrain.getState().Pose.getRotation().getDegrees();
        double fieldRelativeAngle = cumulativeAngle + robotHeading;
        return normalizeAngle(fieldRelativeAngle);
    }

    /**
     * Check and update alliance color from Driver Station.
     */
    public void checkDS() {
        if (!isBlue && !isRed) {
            if (DriverStation.isDSAttached()) {
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue;
                isRed = DriverStation.getAlliance().get() == Alliance.Red;
            }
        }
    }

    /**
     * Returns the current 3D pose of the turret in field coordinates.
     */
    public Pose3d getTurretPose3d() {
        Pose2d robotPose = drivetrain.getState().Pose;
        
        // Convert robot pose to 3D
        Pose3d robotPose3d = new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );
        
        // Turret offset from robot center
        Translation3d turretTranslation = new Translation3d(
            Constants.Turret.offsetX,
            Constants.Turret.offsetY,
            Constants.Turret.offsetZ
        );
        
        // Turret rotation (field-relative)
        Rotation2d fieldAngle = new Rotation2d(Math.toRadians(getFieldAngle()));
        Rotation3d turretRotation = new Rotation3d(0, 0, fieldAngle.getRadians());
        
        // Combine robot pose + turret offset + turret rotation
        Transform3d turretTransform = new Transform3d(turretTranslation, turretRotation);
        return robotPose3d.transformBy(turretTransform);
    }

    @Override
    public void periodic() {
        checkDS();
        
        // CRITICAL: Update cumulative angle BEFORE commands execute
        if (RobotBase.isSimulation()) {
            updateSimulatedEncoder();
        } else {
            updateCumulativeAngle();
        }
        
        // Command the turret
        setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        
        // Logging
        SmartDashboard.putNumber("Turret/CumulativeAngle", getCumulativeAngle());
        SmartDashboard.putNumber("Turret/FieldAngle", getFieldAngle());
    }

    @Override
    public void simulationPeriodic() {
        // Additional simulation logging
        SmartDashboard.putNumber("Turret/SimFieldAngle", simulatedFieldAngle);
        SmartDashboard.putNumber("Turret/SimEncoderReading", prevAbsolute);
    }
}