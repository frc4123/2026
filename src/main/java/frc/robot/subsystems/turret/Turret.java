package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.ShotCache;
import frc.robot.utils.ShotHelper;
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
    private final TalonFX turretMotor = new TalonFX(Constants.CanIdCanivore.TURRET, Constants.CanIdCanivore.CARNIVORE);

    // Absolute turret encoder
    private final CANcoder turretEncoder1 = new CANcoder(Constants.CanIdCanivore.TURRET_ENCODER_1,
            Constants.CanIdCanivore.CARNIVORE);
    private final CANcoder turretEncoder2 = new CANcoder(Constants.CanIdCanivore.TURRET_ENCODER_2,
            Constants.CanIdCanivore.CARNIVORE);

    private boolean hasAbsoluteZero = false;

    private double targetCumulative = 0;

    private double initOffsetDegrees = 0.0; // Encoder rotations from zero at boot

    private int bootDelayCounter = 0;

    private final EasyCRT easyCrtSolver;

    // Motion Magic controller object
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic = new DynamicMotionMagicTorqueCurrentFOC(
            TurretConstants.STOW_POSITION,
            TurretConstants.VELOCITY,
            TurretConstants.ACCELERATION);

    // Make sure these are initialized in your constructor:
    // private final StatusSignal<Angle> motorPositionSignal =
    // turretMotor.getPosition();
    // private final StatusSignal<AngularVelocity> motorVelocitySignal =
    // turretMotor.getVelocity();
    // private final StatusSignal<Voltage> voltageSignal =
    // turretMotor.getMotorVoltage();

    private final StatusSignal<Angle> encoder1PositionSignal = this.turretEncoder1.getPosition();

    private final StatusSignal<Angle> encoder1AbsolutePositionSignal = this.turretEncoder1.getAbsolutePosition();
    private final StatusSignal<Angle> encoder2AbsolutePositionSignal = this.turretEncoder2.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> turretVelocity = this.turretMotor.getVelocity();
    // private final StatusSignal<AngularVelocity> encoderVelocitySignal =
    // turretEncoder1.getVelocity();
    // Physical turret limits relative to turret zero

    // Cumulative turret angle tracking
    private double cumulativeAngle;
    private double simulatedAngle = 0.0; // sim version of cumulativeAngle

    // Swerve reference for heading and yaw rate
    private final CommandSwerveDrivetrain drivetrain;

    // Timing for accel estimation if needed later
    // private double lastLoopTime = 0.0;

    public Turret(final CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        this.configureMotor();
        this.configureCANcoders();

        this.refreshStatusSignals(); // Add this line!
        Timer.delay(0.1);

        this.easyCrtSolver = this.initCRT();

        // Read encoder once at startup
        // double initial = turretEncoder1.getAbsolutePosition().getValueAsDouble();

        // lastLoopTime = Timer.getFPGATimestamp();

        if (Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.SIM) {
            // Simulation setup
            this.hasAbsoluteZero = true;
            this.cumulativeAngle = 0.0;
            this.simulatedAngle = 0.0;
            this.initOffsetDegrees = 0.0;
        }
    }

    private void configureMotor() {
        this.turretMotor.setNeutralMode(NeutralModeValue.Coast);

        final TalonFXConfiguration feedbackUnits = new TalonFXConfiguration();

        feedbackUnits.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackUnits.Feedback.FeedbackRemoteSensorID = Constants.CanIdCanivore.TURRET_ENCODER_1;
        feedbackUnits.Feedback.FeedbackRotorOffset = 0;

        feedbackUnits.Feedback.RotorToSensorRatio = TurretConstants.ROTOR_TO_ENCODER_1_RATION;
        feedbackUnits.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

        // Configure the rest of your motor settings
        feedbackUnits.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feedbackUnits.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        final Slot0Configs pid = new Slot0Configs()
                .withKP(TurretConstants.P)
                .withKI(TurretConstants.I)
                .withKD(TurretConstants.D)
                .withKS(TurretConstants.S)
                .withKV(TurretConstants.V)
                .withKA(TurretConstants.A)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        final MotorOutputConfigs motorOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive);

        final TorqueCurrentConfigs torqueDeadband = new TorqueCurrentConfigs()
                .withTorqueNeutralDeadband(4);

        this.turretMotor.getConfigurator().apply(feedbackUnits);
        this.turretMotor.getConfigurator().apply(pid);
        this.turretMotor.getConfigurator().apply(motorOutput);
        this.turretMotor.getConfigurator().apply(torqueDeadband);
    }

    private void configureCANcoders() {
        // Configure CANcoder 1
        final MagnetSensorConfigs magnetConfig1 = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1.0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) // TODO: check which is which
                .withMagnetOffset(TurretConstants.ENCODER_1_OFFSET); // Set the offset here

        final CANcoderConfiguration config1 = new CANcoderConfiguration()
                .withMagnetSensor(magnetConfig1);

        this.turretEncoder1.getConfigurator().apply(config1);

        // Configure CANcoder 2
        final MagnetSensorConfigs magnetConfig2 = new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1.0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive) // TODO: check which is which
                .withMagnetOffset(TurretConstants.ENCODER_2_OFFSET); // Set the offset here

        final CANcoderConfiguration config2 = new CANcoderConfiguration()
                .withMagnetSensor(magnetConfig2);

        this.turretEncoder2.getConfigurator().apply(config2);
    }

    // Call this once per periodic loop to refresh all signals
    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                this.// motorPositionSignal,
                // motorVelocitySignal,
                // voltageSignal,
                        encoder1AbsolutePositionSignal,
                this.encoder2AbsolutePositionSignal,
                this.encoder1PositionSignal,
                this.turretVelocity// ,
        // encoderVelocitySignal
        );
    }

    private double normalizeAngle(double deg) {
        deg %= 360.0;
        if (deg > 180)
            deg -= 360;
        if (deg < -180)
            deg += 360;
        return deg;
    }

    public EasyCRT initCRT() {

        final Supplier<Angle> enc1Supplier = () -> Units.Rotations
                .of(this.encoder1AbsolutePositionSignal.getValueAsDouble());
        final Supplier<Angle> enc2Supplier = () -> Units.Rotations
                .of(this.encoder2AbsolutePositionSignal.getValueAsDouble());

        final var easyCrt = new EasyCRTConfig(enc1Supplier, enc2Supplier)
                .withEncoderRatios(TurretConstants.SENSOR_TO_MECHANISM_RATIO,
                        TurretConstants.SENSOR_2_TO_MECHANISM_RATIO)
                .withAbsoluteEncoderOffsets(Units.Rotations.of(TurretConstants.ENCODER_1_CRT_OFFSET),
                        Units.Rotations.of(TurretConstants.ENCODER_2_CRT_OFFSET)) // WE ALREADY FLASHED OFFSETS
                .withMechanismRange(Units.Rotations.of(TurretConstants.MECHANISM_MIN_RANGE - 0.07),
                        Units.Rotations.of(TurretConstants.MECHANISM_MAX_RANGE + 0.07))
                // TODO ~1.08 deg at encoder2 for the example ratio im not sure about this so
                // prolly js keep tts as it is or research
                .withMatchTolerance(Units.Rotations.of(0.06))
                .withAbsoluteEncoderInversions(false, false);
        // .withCrtGearRecommendationConstraints(
        // /* coverageMargin */ TurretConstants.coverageMargin,
        // /* minTeeth */ TurretConstants.minTeeth,
        // /* maxTeeth */ TurretConstants.maxTeeth,
        // /* maxIterations */ TurretConstants.maxIterations);

        // you can inspect:
        // easyCrt.getUniqueCoverage(); // Optional<Angle> coverage from prime counts
        // and common scale
        // easyCrt.coverageSatisfiesRange(); // Does coverage exceed maxMechanismAngle?
        // easyCrt.getRecommendedCrtGearPair(); // Suggested pair within constraints
        // easyCrt.getUniqueCoverage();

        // Create the solver:
        return new EasyCRT(easyCrt);
    }

    private void tryResolveAbsolute() {
        if (this.hasAbsoluteZero)
            return;

        // Wait 40 loops (~800ms) before trying to resolve
        if (this.bootDelayCounter < 100) {
            this.bootDelayCounter++;
            return;
        }

        BaseStatusSignal.refreshAll(this.encoder1AbsolutePositionSignal, this.encoder2AbsolutePositionSignal,
                this.encoder1PositionSignal);

        if (!this.encoder1AbsolutePositionSignal.getStatus().isOK()
                || !this.encoder2AbsolutePositionSignal.getStatus().isOK()) {
            System.out.println("Waiting for encoder signals...");
            return;
        }

        final var angleOpt = this.easyCrtSolver.getAngleOptional();
        if (angleOpt.isEmpty())
            return;

        final Angle mechAngle = angleOpt.get();

        this.cumulativeAngle = mechAngle.in(Units.Degrees);

        // Constrain to -360° to +360°
        while (this.cumulativeAngle > 360.0)
            this.cumulativeAngle -= 360.0;
        while (this.cumulativeAngle < -360.0)
            this.cumulativeAngle += 360.0;

        // Current encoder reading using THE SAME MATH as updateCumulativeAngle()
        final double currentEncoderDegrees = this.encoder1PositionSignal.getValueAsDouble() * 360.0
                / TurretConstants.SENSOR_TO_MECHANISM_RATIO;

        // Calculate offset: 87° - 3° = 84° offset
        this.initOffsetDegrees = this.cumulativeAngle - currentEncoderDegrees;

        this.hasAbsoluteZero = true;
    }

    /**
     * Unwrap absolute encoder into cumulative turret angle.
     * Call exactly once per loop.
     */
    private void updateCumulativeAngle() {
        // Get total rotations from encoder
        this.cumulativeAngle = this.initOffsetDegrees
                + (this.encoder1PositionSignal.getValueAsDouble() * 360.0 / TurretConstants.SENSOR_TO_MECHANISM_RATIO);
    }

    public Rotation2d targetAngle(final Pose2d robotPose) {
        final Translation2d target = ShotCache.get().getTarget().toTranslation2d();
        return this.getAngleToTarget(robotPose, target);
    }

    private Rotation2d getAngleToTarget(final Pose2d robotPose, final Translation2d target) {

        final Pose2d turretPose = robotPose.plus(TurretConstants.ROBOT_TO_TURRET_TRANSFORM);

        final Translation2d delta = target.minus(turretPose.getTranslation());
        return delta.getAngle();
    }

    public void setFieldAngle(final Rotation2d targetFieldAngle) {
        double targetRotations = 0;

        // Clamp vision offset
        // cameraOffset = Math.max(-2.0, Math.min(2.0, cameraOffset));

        // Robot heading and yaw rate
        final Rotation2d robotHeading = this.drivetrain.getState().Pose.getRotation();
        // TODO THIS IS WHAT I JS TOOK OUT double targetTurretAngle =
        // normalizeAngle(targetFieldAngle.minus(robotHeading).getDegrees());

        // Compute shortest delta to target
        final double current = this.cumulativeAngle;

        final double robotYawRateDegPerSec = this.drivetrain.getState().Speeds.omegaRadiansPerSecond * 180.0 / Math.PI;
        final double predictionTime = 0.15;
        final Rotation2d predictedRobotHeading = robotHeading.plus(
                Rotation2d.fromDegrees(robotYawRateDegPerSec * predictionTime));
        final double targetTurretAngle = this
                .normalizeAngle(targetFieldAngle.minus(predictedRobotHeading).getDegrees());

        final double delta = this.normalizeAngle(targetTurretAngle - current);

        // Compute new cumulative setpoint
        this.targetCumulative = this.cumulativeAngle + delta; // + cameraOffset;

        // Clamp to physical limits
        while (this.targetCumulative > TurretConstants.MAX_CUMULATIVE_ANGLE) {
            this.targetCumulative -= 360.0;
            ShotHelper.isWrapping(true);
        }
        while (this.targetCumulative < TurretConstants.MIN_CUMULATIVE_ANGLE) {
            this.targetCumulative += 360.0;
            ShotHelper.isWrapping(true);
        }
        // targetCumulative = Math.max(minCumulativeAngle, Math.min(maxCumulativeAngle,
        // targetCumulative));

        // Convert position target to motor rotations
        targetRotations = (this.targetCumulative - this.initOffsetDegrees) / 360.0;

        // Command Motion Magic with combined velocity feedforward
        // SmartDashboard.putNumber("TargetAngle", targetRotations * 360);
        SmartDashboard.putNumber("MotionMagicAngle ", (targetRotations * 360) + this.initOffsetDegrees);
        this.turretMotor.setControl(
                this.motionMagic
                        .withPosition(targetRotations)
        // .withFeedForward(feedforwardVolts)
        // .withFeedForward(0)
        );
    }

    public double getCumulativeAngle() {
        if (Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.SIM) {
            return this.simulatedAngle;
        }
        return this.cumulativeAngle;
    }

    public double getFieldAngle() {
        final double robotHeading = this.drivetrain.getState().Pose.getRotation().getDegrees();
        final double fieldRelativeAngle = this.cumulativeAngle + robotHeading;
        return this.normalizeAngle(fieldRelativeAngle);
    }

    @Override
    public void periodic() {
        this.refreshStatusSignals();
        if (this.hasAbsoluteZero) {
            this.updateCumulativeAngle();
        }
        if (!this.hasAbsoluteZero) {
            this.tryResolveAbsolute();
            this.turretMotor.stopMotor();
            return;
        }

        if (ShotHelper.getIsWrapping() && this.turretVelocity.getValueAsDouble() < 0.5) {
            ShotHelper.isWrapping(false);
        } // TODO PLOT VELOCITY WHEN SPINNING AND FIND THE SWEET SPOT

        SmartDashboard.putNumber("Turret CumulativeAngle", this.getCumulativeAngle());
        SmartDashboard.putNumber("Turret Target Angle ", this.targetCumulative);

        // SmartDashboard.putNumber("Encoder1 Position",
        // encoder1AbsolutePositionSignal.getValueAsDouble());
        // SmartDashboard.putNumber("Encoder2 Position",
        // encoder2AbsolutePositionSignal.getValueAsDouble());
        // SmartDashboard.putNumber("Motor Pos (rot)",
        // turretMotor.getPosition().getValueAsDouble() * 360);
        // SmartDashboard.putNumber("initoffset", initOffsetDegrees);
        // SmartDashboard.putNumber("CRT Angle",
        // easyCrtSolver.getAngleOptional().orElse(Rotations.of(0)).in(Units.Degrees));
        // SmartDashboard.putNumber("closedlooperror",
        // turretMotor.getClosedLoopError().getValueAsDouble());

    }

    @Override
    public void simulationPeriodic() {
        // Don't call updateCumulativeAngle() - we're simulating it here

        // Get commanded position
        final double commandedRotations = this.motionMagic.Position;
        final double commandedDegrees = commandedRotations * 360.0 + this.initOffsetDegrees;

        // Simulate motor movement with slew rate
        final double step = 25.0; // degrees per 20ms
        final double diff = commandedDegrees - this.simulatedAngle;

        if (Math.abs(diff) > step) {
            this.simulatedAngle += Math.copySign(step, diff);
        } else {
            this.simulatedAngle = commandedDegrees;
        }

        // Update cumulativeAngle to match simulation
        this.cumulativeAngle = this.simulatedAngle;

        // Command the turret for NEXT loop
        this.setFieldAngle(this.targetAngle(this.drivetrain.getState().Pose));

        SmartDashboard.putNumber("Turret Angle (Sim)", this.simulatedAngle);
        SmartDashboard.putNumber("Turret Commanded", commandedDegrees);
    }
}