package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.TurretConstants;

/**
 * Turret subsystem for field-relative aiming.
 * Tracks cumulative angle using CANCoder with unwrapping.
 * Physical range ±360 degrees from turret zero.
 * Uses velocity feedforward to cancel robot yaw motion.
 */
public class Turret extends SubsystemBase {

    CANBus canivore = new CANBus(Constants.CanIdCanivore.ID);
    // Motor controlling turret rotation
    private final TalonFX turretMotor = new TalonFX(Constants.CanIdCanivore.Turret, canivore);

    // Absolute turret encoder
    private final CANcoder turretEncoder1 = new CANcoder(Constants.CanIdCanivore.Turret_Encoder1, canivore);
    private final CANcoder turretEncoder2 = new CANcoder(Constants.CanIdCanivore.Turret_Encoder2, canivore);

    private static boolean isBlue = false;
    private static boolean isRed = false;
    // get alliance color

    private boolean hasAbsoluteZero = false;


    private EasyCRT easyCrtSolver = initCRT();

    // Motion Magic controller object
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic =
            new DynamicMotionMagicTorqueCurrentFOC(
                    TurretConstants.stowPosition,
                    TurretConstants.velocity,
                    TurretConstants.acceleration
            );

    // Make sure these are initialized in your constructor:
    private final StatusSignal<Angle> motorPositionSignal = turretMotor.getPosition();
    private final StatusSignal<AngularVelocity> motorVelocitySignal = turretMotor.getVelocity();
    private final StatusSignal<Voltage> voltageSignal = turretMotor.getMotorVoltage();

    private final StatusSignal<Angle> encoderPositionSignal = turretEncoder1.getPosition();
    private final StatusSignal<AngularVelocity> encoderVelocitySignal = turretEncoder1.getVelocity();


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
        configureCANcoders();
        initCRT();

        // Read encoder once at startup
        // double initial = turretEncoder1.getAbsolutePosition().getValueAsDouble();
        cumulativeAngle = easyCrtSolver.getAngleOptional().get().in(Units.Degrees);
;
        prevAbsolute = cumulativeAngle;
        //lastLoopTime = Timer.getFPGATimestamp();

        
    }

    private void configureMotor() {
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = Constants.CanIdCanivore.Turret_Encoder1; // ID of CANcoder

        config.Feedback.RotorToSensorRatio = TurretConstants.rotorToEncoder1Ratio;    
        config.Feedback.SensorToMechanismRatio = TurretConstants.sensorToMechanismRatio;
        
        // Configure the rest of your motor settings
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        turretMotor.getConfigurator().apply(config);

        Slot0Configs pid = new Slot0Configs()
                .withKP(TurretConstants.kP)
                .withKI(TurretConstants.kI)
                .withKD(TurretConstants.kD)
                .withKV(TurretConstants.kV)
                .withKA(TurretConstants.kA);

        turretMotor.getConfigurator().apply(pid);


    }

    private void configureCANcoders() {
    // Configure CANcoder 1
    MagnetSensorConfigs magnetConfig1 = new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1.0)
        //.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) TODO: check which is which
        .withMagnetOffset(TurretConstants.encoder1Offset); // Set the offset here
    
    CANcoderConfiguration config1 = new CANcoderConfiguration()
        .withMagnetSensor(magnetConfig1);
    
    turretEncoder1.getConfigurator().apply(config1);
    
    // Configure CANcoder 2
    MagnetSensorConfigs magnetConfig2 = new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1.0)
        //.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) TODO: check which is which
        .withMagnetOffset(TurretConstants.encoder2Offset); // Set the offset here
    
        CANcoderConfiguration config2 = new CANcoderConfiguration()
        .withMagnetSensor(magnetConfig2);
    
        turretEncoder2.getConfigurator().apply(config2);
    }

    // Call this once per periodic loop to refresh all signals
    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(motorPositionSignal, motorVelocitySignal, voltageSignal, encoderPositionSignal, encoderVelocitySignal);
    }


    private double degreesToMotorRotations(double deg) {
        return deg / 360.0 * TurretConstants.motorToTurretRatio;
    }

    private double normalizeAngle(double deg) {
        deg %= 360.0;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }

    public EasyCRT initCRT(){
        // Suppose: mechanism : drive gear = 12:1, drive gear = 50T, encoders use 19T and 23T pinions.

        Supplier<Angle> enc1Supplier = () -> Rotations.of(turretEncoder1.getPosition().getValueAsDouble());
        Supplier<Angle> enc2Supplier = () -> Rotations.of(turretEncoder2.getPosition().getValueAsDouble());

        var easyCrt =
            new EasyCRTConfig(enc1Supplier, enc2Supplier)
                .withEncoderRatios(TurretConstants.turretGearTeeth / TurretConstants.encoder1Teeth, TurretConstants.turretGearTeeth / TurretConstants.encoder2Teeth)
                .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(0)) // WE ALREADY FLASHED OFFSETS
                .withMechanismRange(Rotations.of(TurretConstants.mechanismMinRange), Rotations.of(TurretConstants.mechanismMaxRange)) // -360 deg to +720 deg
                .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio im not sure about this so prolly js keep tts as it is or research //TODO: research
                .withAbsoluteEncoderInversions(false, false)
                .withCrtGearRecommendationConstraints(
                    /* coverageMargin */ TurretConstants.coverageMargin,
                    /* minTeeth */ TurretConstants.minTeeth,
                    /* maxTeeth */ TurretConstants.maxTeeth,
                    /* maxIterations */ TurretConstants.maxIterations);
                

        // you can inspect:
        easyCrt.getUniqueCoverage();          // Optional<Angle> coverage from prime counts and common scale
        easyCrt.coverageSatisfiesRange();     // Does coverage exceed maxMechanismAngle?
        easyCrt.getRecommendedCrtGearPair(); // Suggested pair within constraints
        easyCrt.getUniqueCoverage();

        // Create the solver:
        var easyCrtSolver = new EasyCRT(easyCrt);
        return easyCrtSolver;
    }

    private void tryResolveAbsolute() {
        if (hasAbsoluteZero) return;

        var angleOpt = easyCrtSolver.getAngleOptional();
        if (angleOpt.isEmpty()) return;

        Angle mechAngle = angleOpt.get();

        cumulativeAngle = mechAngle.in(Units.Degrees);
        prevAbsolute = cumulativeAngle;

        hasAbsoluteZero = true;
    }


    /**
     * Unwrap absolute encoder into cumulative turret angle.
     * Call exactly once per loop.
     */
    private void updateCumulativeAngle() {
        // Get total rotations from encoder
        cumulativeAngle = encoderPositionSignal.getValueAsDouble() * 360;
    }

    public Rotation2d targetAngle(Pose2d robotPose) {

        Translation2d target;
        Pose3d blueHub = VisionConstants.blueHub;
        Pose3d redHub = VisionConstants.redHub;

        if(isBlue && robotPose.getX() < blueHub.getX()){
            target = VisionConstants.blueHub.getTranslation().toTranslation2d();
            Translation2d robotPos = robotPose.getTranslation();
        
           Translation2d turretPos = robotPos.plus(
                TurretConstants.turretOffset.rotateBy(robotPose.getRotation())
            );

            Translation2d delta = target.minus(turretPos);
            return delta.getAngle();

        } else if (isRed && robotPose.getX() > redHub.getX()){
            target = VisionConstants.redHub.getTranslation().toTranslation2d();
            Translation2d robotPos = robotPose.getTranslation();

            Translation2d turretPos = robotPos.plus(
                TurretConstants.turretOffset.rotateBy(robotPose.getRotation())
            );

            Translation2d delta = target.minus(turretPos);
            return delta.getAngle();
        }
        /*this should allow the robot to face the hub from whatever position it is
        we will use this comma fnd if our turret breaks and we havfe to start auto aiming using swerve and not turret 
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
        
        // Same target selection logic as targetAngle
        Translation2d target;
        Pose3d blueHub = VisionConstants.blueHub;
        Pose3d redHub = VisionConstants.redHub;

        if(isBlue && robotPose.getX() < blueHub.getX()){
            target = VisionConstants.blueHub.getTranslation().toTranslation2d();
        } else if (isRed && robotPose.getX() > redHub.getX()){
            target = VisionConstants.redHub.getTranslation().toTranslation2d();
        } else {
            // No valid target, return 0 feedforward
            return 0.0;
        }
        
        // Account for turret offset from robot center (same as targetAngle)
        Translation2d robotPos = robotPose.getTranslation();
        Translation2d turretPos = robotPos.plus(
            TurretConstants.turretOffset.rotateBy(robotPose.getRotation())
        );
                
        // Vector from turret to target
        Translation2d toTarget = target.minus(turretPos);
        
        double distanceSquared = toTarget.getNorm() * toTarget.getNorm();
        
        if (distanceSquared < 0.0001) {
            return 0.0; // Avoid division by zero when very close
        }
        
        // Get robot velocity in field frame
        var robotSpeeds = drivetrain.getState().Speeds;
        
        // Cross product gives angular velocity (rad/s)
        double crossProduct = robotSpeeds.vxMetersPerSecond * toTarget.getY() - 
                            robotSpeeds.vyMetersPerSecond * toTarget.getX();
        
        double angularVelocity_radPerSec = crossProduct / distanceSquared;
        
        // Convert to degrees per second (to match your rotation FF units)
        return angularVelocity_radPerSec * 180.0 / Math.PI;
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
        double totalFF_rotPerSec = totalFF_degPerSec / 360.0 * TurretConstants.motorToTurretRatio;

        // Convert position target to motor rotations
        double targetRotations = degreesToMotorRotations(targetCumulative);

        // Command Motion Magic with combined velocity feedforward
        turretMotor.setControl(
                motionMagic
                        .withPosition(targetRotations)
                        .withFeedForward(totalFF_rotPerSec)
            );
        }

    public double getCumulativeAngle() {
        if(Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.Sim) {
            return simulatedAngle;
        }
        return cumulativeAngle;
    }

    public double getFieldAngle() {
        double robotHeading = drivetrain.getState().Pose.getRotation().getDegrees();
        double fieldRelativeAngle = cumulativeAngle + robotHeading;
        return normalizeAngle(fieldRelativeAngle);
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
    // public Pose3d getTurretPose3d() {
    //     // Get robot pose
    //     Pose2d robotPose = drivetrain.getState().Pose;
        
    //     // Convert to 3D (robot is on the ground)
    //     Pose3d robotPose3d = new Pose3d(
    //         robotPose.getX(),
    //         robotPose.getY(),
    //         0.0, // z position (on field)
    //         new Rotation3d(0, 0, robotPose.getRotation().getRadians())
    //     );
        
    //     // Turret offset from robot center
    //     Translation3d turretTranslation = new Translation3d(
    //         TurretConstants.offsetX,
    //         TurretConstants.offsetY,
    //         TurretConstants.offsetZ // You'll need to add this constant 
    //     );
        
    //     // Turret rotation (field-relative)
    //     Rotation2d fieldAngle = new Rotation2d(Math.toRadians(
    //         normalizeAngle(cumulativeAngle) + robotPose.getRotation().getDegrees()
    //     ));
        
    //     Rotation3d turretRotation = new Rotation3d(0, 0, fieldAngle.getRadians());
        
    //     // Combine: robot pose + turret offset + turret rotation
    //     Transform3d turretTransform = new Transform3d(turretTranslation, turretRotation);
        
    //     return robotPose3d.transformBy(turretTransform);
    // }

    @Override
    public void periodic() {
        updateCumulativeAngle();
        checkDS();
        refreshStatusSignals();
        setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        if (!hasAbsoluteZero) {
            tryResolveAbsolute();
            turretMotor.stopMotor();
            return;
        }

        SmartDashboard.putNumber("Turret Angle", getCumulativeAngle());
    }

   @Override
    public void simulationPeriodic() {
        // FIRST: Simulate motor response
        updateCumulativeAngle();
        double commandedRotations = motionMagic.Position;
        double commandedDegrees = commandedRotations / TurretConstants.motorToTurretRatio * 360.0;
        
        double step = 25.0;
        double diff = commandedDegrees - simulatedAngle;
        
        if (Math.abs(diff) > step) {
            simulatedAngle += Math.copySign(step, diff);
        } else {
            simulatedAngle = commandedDegrees;
        }
        
        // THEN: Update cumulative tracking
        double absDegrees = simulatedAngle % 360.0;
        if (absDegrees < 0) absDegrees += 360.0;
        
        double delta = absDegrees - prevAbsolute;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        
        cumulativeAngle += delta;
        prevAbsolute = absDegrees;
        
        // FINALLY: Command the turret for NEXT loop
        setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        
        SmartDashboard.putNumber("Turret Angle (Sim)", simulatedAngle);
        SmartDashboard.putNumber("Turret Commanded", commandedDegrees);
    }
}