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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
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
    // Motor controlling turret rotation
    private final TalonFX turretMotor = new TalonFX(Constants.CanIdCanivore.Turret, canivore);

    // Absolute turret encoder
    private final CANcoder turretEncoder = new CANcoder(Constants.CanIdCanivore.Turret_Encoder);


    private static boolean isBlue = false;
    private static boolean isRed = false;
    // get alliance color

    // Motion Magic controller object
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

    // Cumulative turret angle tracking
    private double cumulativeAngle;
    private double simulatedAngle = 0.0; // sim version of cumulativeAngle

    private double prevAbsolute;

    // Swerve reference for heading and yaw rate
    private final CommandSwerveDrivetrain drivetrain;
    private final Vision vision;

    // Timing for accel estimation if needed later
    //private double lastLoopTime = 0.0;

    public Turret(CommandSwerveDrivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        configureMotor();
        configureEncoder();

        // Read encoder once at startup
        double initial = turretEncoder.getAbsolutePosition().getValueAsDouble();
        cumulativeAngle = initial;
        prevAbsolute = initial;

        //lastLoopTime = Timer.getFPGATimestamp();
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
        //turretEncoder.configFactoryDefault();
        /*turretEncoder.configSensorInitializationStrategy(
                com.ctre.phoenix.sensors.SensorInitializationStrategy.BootToAbsolutePosition
        );*/
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

    public Rotation2d targetAngle(Pose2d robotPose) {

        Translation2d target;
        Pose3d blueHub = Constants.VisionConstants.blueHub;
        Pose3d redHub = Constants.VisionConstants.redHub;

        if(isBlue && robotPose.getX() < blueHub.getX()){
            target = Constants.VisionConstants.blueHub.getTranslation().toTranslation2d();
            Translation2d robotPos = robotPose.getTranslation();
        
            Translation2d turretPos = robotPos.plus(Constants.Turret.turretOffset);

            Translation2d delta = target.minus(turretPos);
            return delta.getAngle();

        } else if (isRed && robotPose.getX() > redHub.getX()){
            target = Constants.VisionConstants.redHub.getTranslation().toTranslation2d();
            Translation2d robotPos = robotPose.getTranslation();

            Translation2d turretPos = robotPos.plus(Constants.Turret.turretOffset);

            Translation2d delta = target.minus(turretPos);
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

        /**
     * Calculate feedforward for robot translation causing angle to target to change.
     * Mimics the same target calculation logic as targetAngle.
     */
    private double calculateTranslationFeedforward() {
        Pose2d robotPose = drivetrain.getState().Pose;
        
        // Target selection
        Translation2d target;
        Pose3d blueHub = Constants.VisionConstants.blueHub;
        Pose3d redHub = Constants.VisionConstants.redHub;

        if(isBlue && robotPose.getX() < blueHub.getX()){
            target = blueHub.getTranslation().toTranslation2d();
        } else if (isRed && robotPose.getX() > redHub.getX()){
            target = redHub.getTranslation().toTranslation2d();
        } else {
            return 0.0;
        }
        
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d turretPos = robotPos.plus(Constants.Turret.turretOffset);
        
        // Vector from turret to target (FIELD frame)
        Translation2d r = target.minus(turretPos);
        
        double distanceSquared = r.getNorm() * r.getNorm();
        if (distanceSquared < 0.0001) return 0.0;
        
        // Get FIELD-relative velocity (what you already have)
        var fieldSpeeds = drivetrain.getState().Speeds;
        
        // Cross product in FIELD frame (both r and v are field-relative)
        // ω = (v_field × r_field) / |r|²
        double cross = fieldSpeeds.vxMetersPerSecond * r.getY() - 
                    fieldSpeeds.vyMetersPerSecond * r.getX();
        
        double omega_rad_per_sec = cross / distanceSquared;
        
        return omega_rad_per_sec * 180.0 / Math.PI;
    }

    /**
     * Field-relative turret control with yaw velocity feedforward.
     */
    public void setFieldAngle(Rotation2d targetFieldAngle, double cameraOffset) {

        // Clamp vision offset
        cameraOffset = Math.max(-1.0, Math.min(1.0, cameraOffset));

        // Robot heading and yaw rate
        Rotation2d robotHeading = drivetrain.getState().Pose.getRotation();
        double robotYawRateDegPerSec = drivetrain.getState().Speeds.omegaRadiansPerSecond / Math.PI * 180;

        // Convert field target into robot-relative turret target
        double targetTurretAngle = normalizeAngle(targetFieldAngle.minus(robotHeading).getDegrees());

        // Compute shortest delta to target
        double current = cumulativeAngle;
        double delta = normalizeAngle(targetTurretAngle - current);

        // Compute new cumulative setpoint
        double targetCumulative = cumulativeAngle + delta + cameraOffset;

        // Clamp to physical limits 
        if (targetCumulative > maxCumulativeAngle) {
            targetCumulative -= 360.0;  // Wrap from 361° to -359°
        } else if (targetCumulative < minCumulativeAngle) {
            targetCumulative += 360.0;  // Wrap from -361° to 359°
        }
        targetCumulative = Math.max(minCumulativeAngle, Math.min(maxCumulativeAngle, targetCumulative));

        targetCumulative = Math.max(minCumulativeAngle, Math.min(maxCumulativeAngle, targetCumulative));
        //clamp in case

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
    

    public double getCumulativeAngle() {
        if(Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.Sim) {
            return simulatedAngle;
        }
        return cumulativeAngle;
    }

    public double getAbsoluteAngle() {
        return turretEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getFieldAngle() {
        return normalizeAngle(cumulativeAngle);
    }

    public void checkDS(){
        if(isBlue == false && isRed == false){
            if(DriverStation.isDSAttached()){
                isBlue = DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
                isRed = DriverStation.getAlliance().get() == Alliance.Red ? true : false;
            } else {
                isBlue = false;
                isRed = false;
            }
        }
    }

    /**
     * Returns the current 3D pose of the turret in field coordinates.
     * Combines robot position with turret rotation.
     */
    public Pose3d getTurretPose3d() {
        // Get robot pose
        Pose2d robotPose = drivetrain.getState().Pose;
        
        // Convert to 3D (robot is on the ground)
        Pose3d robotPose3d = new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0, // z position (on field)
            new Rotation3d(0, 0, robotPose.getRotation().getRadians())
        );
        
        // Turret offset from robot center
        Translation3d turretTranslation = new Translation3d(
            Constants.Turret.offsetX,
            Constants.Turret.offsetY,
            Constants.Turret.offsetZ // You'll need to add this constant
        );
        
        // Turret rotation (field-relative)
        Rotation2d fieldAngle = new Rotation2d(Math.toRadians(
            normalizeAngle(cumulativeAngle) + robotPose.getRotation().getDegrees()
        ));
        
        Rotation3d turretRotation = new Rotation3d(0, 0, fieldAngle.getRadians());
        
        // Combine: robot pose + turret offset + turret rotation
        Transform3d turretTransform = new Transform3d(turretTranslation, turretRotation);
        
        return robotPose3d.transformBy(turretTransform);
    }

    @Override
    public void periodic() {
        checkDS();
        if (!RobotBase.isSimulation()) {updateCumulativeAngle();}
        setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        SmartDashboard.putNumber("Turret Angle", getCumulativeAngle());
    }

    @Override
    public void simulationPeriodic() {
        // Simulate encoder reading (0-360°)
        double absDegrees = simulatedAngle % 360.0;
        if (absDegrees < 0) absDegrees += 360.0;
        
        // Update cumulative tracking (same as real code)
        double delta = absDegrees - prevAbsolute;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        
        cumulativeAngle += delta;
        prevAbsolute = absDegrees;
        
        // Command the turret
        setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        
        // Simulate motor response
        double commandedRotations = motionMagic.Position;
        double commandedDegrees = commandedRotations / gearRatio * 360.0;
        
        double step = 25.0;
        double diff = commandedDegrees - simulatedAngle;
        
        if (Math.abs(diff) > step) {
            simulatedAngle += Math.copySign(step, diff);
        } else {
            simulatedAngle = commandedDegrees;
        }

        
        SmartDashboard.putNumber("Turret Angle (Sim)", simulatedAngle);
        SmartDashboard.putNumber("Turret Commanded", commandedDegrees);
    }
}

