package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.TurretConstants;

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/**
 * Turret subsystem for field-relative aiming.
 * Tracks cumulative angle using CANCoder with unwrapping.
 * Physical range ±360 degrees from turret zero.
 * Uses velocity feedforward to cancel robot yaw motion.
 */
public class Turret extends SubsystemBase {

    // Motor controlling turret rotation
    private final TalonFX turretMotor = new TalonFX(Constants.CanIdCanivore.Turret, Constants.CanIdCanivore.canivore);

    // Absolute turret encoder
    private final CANcoder turretEncoder1 = new CANcoder(Constants.CanIdCanivore.Turret_Encoder1, Constants.CanIdCanivore.canivore);
    private final CANcoder turretEncoder2 = new CANcoder(Constants.CanIdCanivore.Turret_Encoder2, Constants.CanIdCanivore.canivore);

    private static boolean isBlue = false;
    private static boolean isRed = false;
    // get alliance color

    private boolean hasAbsoluteZero = false;

    private double initOffsetDegrees = 0.0; // Encoder rotations from zero at boot

    private double previousYawRate = 0.0;
    private int bootDelayCounter = 0;

    private EasyCRT easyCrtSolver;

    // Motion Magic controller object
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic =
            new DynamicMotionMagicTorqueCurrentFOC(
                TurretConstants.stowPosition,
                TurretConstants.velocity,
                TurretConstants.acceleration
            );

    // Make sure these are initialized in your constructor:
    //private final StatusSignal<Angle> motorPositionSignal = turretMotor.getPosition();
    // private final StatusSignal<AngularVelocity> motorVelocitySignal = turretMotor.getVelocity();
    // private final StatusSignal<Voltage> voltageSignal = turretMotor.getMotorVoltage();

    private final StatusSignal<Angle> encoder1PositionSignal = turretEncoder1.getPosition();
    private final StatusSignal<Angle> encoder2PositionSignal = turretEncoder2.getPosition();
    // private final StatusSignal<AngularVelocity> encoderVelocitySignal = turretEncoder1.getVelocity();
    // Physical turret limits relative to turret zero
    private final double minCumulativeAngle = TurretConstants.mechanismMinRange * 360.0;
    private final double maxCumulativeAngle = TurretConstants.mechanismMaxRange * 360.0;

    // Cumulative turret angle tracking
    private double cumulativeAngle;
    private double simulatedAngle = 0.0; // sim version of cumulativeAngle

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

        refreshStatusSignals();  // Add this line!
        Timer.delay(0.1);  

        easyCrtSolver = initCRT();

        // Read encoder once at startup 
        // double initial = turretEncoder1.getAbsolutePosition().getValueAsDouble();
        
        //lastLoopTime = Timer.getFPGATimestamp();

        if (Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.Sim) {
            // Simulation setup
            hasAbsoluteZero = true;
            cumulativeAngle = 0.0;
            simulatedAngle = 0.0;
            initOffsetDegrees = 0.0;
        } else {
            // Real robot setup
            refreshStatusSignals();
            Timer.delay(0.1);
            easyCrtSolver = initCRT();
        }
    }

    private void configureMotor() {
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration feedbackUnits = new TalonFXConfiguration();

        feedbackUnits.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackUnits.Feedback.FeedbackRemoteSensorID = Constants.CanIdCanivore.Turret_Encoder1; // ID of CANcoder

        feedbackUnits.Feedback.RotorToSensorRatio = TurretConstants.rotorToEncoder1Ratio;    
        feedbackUnits.Feedback.SensorToMechanismRatio = TurretConstants.sensorToMechanismRatio;
        
        // Configure the rest of your motor settings
        feedbackUnits.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feedbackUnits.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        

        Slot0Configs pid = new Slot0Configs()
            .withKP(TurretConstants.kP)
            .withKI(TurretConstants.kI)
            .withKD(TurretConstants.kD)
            .withKS(TurretConstants.kS)
            .withKV(TurretConstants.kV)
            .withKA(TurretConstants.kA);

        turretMotor.getConfigurator().apply(feedbackUnits);
        turretMotor.getConfigurator().apply(pid);
        //turretMotor.getConfigurator().apply(motionMagic);
    }

    private void configureCANcoders() {
        // Configure CANcoder 1
        MagnetSensorConfigs magnetConfig1 = new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(1.0)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive) //TODO: check which is which
            .withMagnetOffset(TurretConstants.encoder1Offset); // Set the offset here
        
        CANcoderConfiguration config1 = new CANcoderConfiguration()
            .withMagnetSensor(magnetConfig1);

        turretEncoder1.getConfigurator().apply(config1);
    
        // Configure CANcoder 2
        MagnetSensorConfigs magnetConfig2 = new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1.0)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive) //TODO: check which is which
        .withMagnetOffset(TurretConstants.encoder2Offset); // Set the offset here
    
        CANcoderConfiguration config2 = new CANcoderConfiguration()
        .withMagnetSensor(magnetConfig2);
    
        turretEncoder2.getConfigurator().apply(config2);
    }

    // Call this once per periodic loop to refresh all signals
    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
        //     motorPositionSignal, 
        //     motorVelocitySignal,
        //     voltageSignal,
            encoder1PositionSignal,
            encoder2PositionSignal//,
        //     encoderVelocitySignal
        );
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
                .withEncoderRatios(TurretConstants.sensorToMechanismRatio, TurretConstants.mechanismGearTeeth / TurretConstants.encoder2Teeth)
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
        if (hasAbsoluteZero) return;
    
        // Wait 40 loops (~800ms) before trying to resolve
        if (bootDelayCounter < 40) {
            bootDelayCounter++;
            return;
        }


        if (!encoder1PositionSignal.getStatus().isOK() || !encoder2PositionSignal.getStatus().isOK()) {
            System.out.println("Waiting for encoder signals...");
            return;
        }

        var angleOpt = easyCrtSolver.getAngleOptional();
        if (angleOpt.isEmpty()) return;

        Angle mechAngle = angleOpt.get();

        cumulativeAngle = mechAngle.in(Units.Degrees);

        // Constrain to -360° to +360°
        while (cumulativeAngle > 360.0) cumulativeAngle -= 360.0;
        while (cumulativeAngle < -360.0) cumulativeAngle += 360.0;

        // Current encoder reading using THE SAME MATH as updateCumulativeAngle()
        double currentEncoderDegrees = encoder1PositionSignal.getValueAsDouble() * 360.0 / TurretConstants.sensorToMechanismRatio;
        
        // Calculate offset
        initOffsetDegrees = cumulativeAngle - currentEncoderDegrees;
        // Calculate offset: 87° - 3° = 84° offset
        initOffsetDegrees = cumulativeAngle - currentEncoderDegrees;


        hasAbsoluteZero = true;
    }


    /**
     * Unwrap absolute encoder into cumulative turret angle.
     * Call exactly once per loop.
     */
    private void updateCumulativeAngle() {
        // Get total rotations from encoder
        cumulativeAngle = initOffsetDegrees + (encoder1PositionSignal.getValueAsDouble() * 360.0 / TurretConstants.sensorToMechanismRatio);
        //cumulativeAngle = encoderDegrees / (TurretConstants.sensorToMechanismRatio);
        //cumulativeAngle = motorPositionSignal.getValueAsDouble() * 360;
        // cumulativeAngle = encoder1PositionSignal.getValueAsDouble() * 360; // original line which had ~~ 7.11 error
    }

    public Rotation2d targetAngle(Pose2d robotPose) {
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
            if(x < VisionConstants.blueHub.getX()){
                return getAngleToTarget(robotPose, VisionConstants.blueHub.getTranslation().toTranslation2d());
            // Check Y zones from top to bottom
            }else if (y >= 5.029) {
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
            if(x > VisionConstants.redHub.getX()){
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

    // Helper method to calculate angle
    private Rotation2d getAngleToTarget(Pose2d robotPose, Translation2d target) {
        Translation2d delta = target.minus(robotPose.getTranslation());
        return delta.getAngle();
    }

    /**
     * Field-relative turret control with yaw velocity feedforward.
     */
    public void setFieldAngle(Rotation2d targetFieldAngle, double cameraOffset) {

        // Clamp vision offset
        cameraOffset = Math.max(-3.0, Math.min(3.0, cameraOffset));

        // Robot heading and yaw rate
        Rotation2d robotHeading = drivetrain.getState().Pose.getRotation();
        double robotYawRateDegPerSec = drivetrain.getState().Speeds.omegaRadiansPerSecond / Math.PI * 180.0;

        // Convert field target into robot-relative turret target
        SmartDashboard.putNumber("Field relative turret target", targetFieldAngle.getDegrees());
        //double targetTurretAngle = normalizeAngle(targetFieldAngle.minus(robotHeading).getDegrees());
        double predictionTime = 0.2; // 200ms - tune this!
        
        // Estimate angular acceleration from previous velocity
        double currentYawRate = robotYawRateDegPerSec;
        double angularAccel = (currentYawRate - previousYawRate) / 0.02; // assuming 20ms loop time

        Rotation2d predictedRobotHeading = robotHeading.plus(
            Rotation2d.fromDegrees(
                robotYawRateDegPerSec * predictionTime + 
                0.5 * angularAccel * predictionTime * predictionTime
            )
        );

        /* REPLACE LINES 387-396 WITH THIS IF IT JITTERS TMR
        Rotation2d predictedRobotHeading = robotHeading.plus(
            Rotation2d.fromDegrees(robotYawRateDegPerSec * predictionTime)
        );

        double targetTurretAngle = normalizeAngle(targetFieldAngle.minus(predictedRobotHeading).getDegrees());
        */

        previousYawRate = robotYawRateDegPerSec;
    
        double targetTurretAngle = normalizeAngle(targetFieldAngle.minus(predictedRobotHeading).getDegrees());
        SmartDashboard.putNumber("Robot Relative Turret Target Rot", targetTurretAngle);

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

        // Convert position target to motor rotations
        double targetRotations = (targetCumulative - initOffsetDegrees) / 360.0;

        // Command Motion Magic with combined velocity feedforward
        SmartDashboard.putNumber("TargetRotations", targetRotations);
        turretMotor.setControl(
                motionMagic
                        .withPosition(targetRotations)
                        //.withFeedForward(feedforwardVolts)
                        //.withFeedForward(0)
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

    @Override
    public void periodic() {
        refreshStatusSignals();
        checkDS();
        if(hasAbsoluteZero) {updateCumulativeAngle();}
        if (!hasAbsoluteZero) {
            tryResolveAbsolute();
            turretMotor.stopMotor();
            return;
        }
        //setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        

        SmartDashboard.putNumber("Turret CumulativeAngle", getCumulativeAngle());
    //     SmartDashboard.putNumber("Encoder1 Position", encoder1PositionSignal.getValueAsDouble());
    //     SmartDashboard.putNumber("Encoder2 Position", encoder2PositionSignal.getValueAsDouble());
    //     SmartDashboard.putNumber("Motor Pos (rot)", motorPositionSignal.getValueAsDouble());
    //     SmartDashboard.putNumber("CRT Angle", easyCrtSolver.getAngleOptional().orElse(Rotations.of(0)).in(Units.Degrees));

    }

   @Override
    public void simulationPeriodic() {
        // Don't call updateCumulativeAngle() - we're simulating it here
        
        // Get commanded position
        double commandedRotations = motionMagic.Position;
        double commandedDegrees = commandedRotations * 360.0 + initOffsetDegrees;
        
        // Simulate motor movement with slew rate
        double step = 25.0; // degrees per 20ms
        double diff = commandedDegrees - simulatedAngle;
        
        if (Math.abs(diff) > step) {
            simulatedAngle += Math.copySign(step, diff);
        } else {
            simulatedAngle = commandedDegrees;
        }
        
        // Update cumulativeAngle to match simulation
        cumulativeAngle = simulatedAngle;
        
        // Command the turret for NEXT loop
        setFieldAngle(targetAngle(drivetrain.getState().Pose), vision.getTurretCamOffset());
        
        SmartDashboard.putNumber("Turret Angle (Sim)", simulatedAngle);
        SmartDashboard.putNumber("Turret Commanded", commandedDegrees);
    }
}