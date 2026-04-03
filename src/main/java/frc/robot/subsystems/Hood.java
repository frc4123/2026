package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.ShotCache;

public class Hood extends SubsystemBase {

    private boolean wasPressed = false;

    private final TalonFX hoodMotor = new TalonFX(
            Constants.CanIdCanivore.HOOD,
            Constants.CanIdCanivore.CARNIVORE);

    // why is this hoodcandi pointing to intakecandi?
    // private static final CANdi HOOD_CANDI = IntakeArmConstants.intakeCANdi;
    private final CANdi candi;
    private final StatusSignal<Boolean> s2Signal;

    private final DynamicMotionMagicTorqueCurrentFOC motionMagic = new DynamicMotionMagicTorqueCurrentFOC(
            HoodConstants.STOW_POSITION,
            HoodConstants.velocity,
            HoodConstants.acceleration);

    private final DynamicMotionMagicTorqueCurrentFOC motionMagicFree = new DynamicMotionMagicTorqueCurrentFOC(
            HoodConstants.STOW_POSITION,
            HoodConstants.slowVelocity,
            HoodConstants.acceleration);

    public Hood(final CANdi candi) {
        this.candi = candi;
        this.configureMotor();
        this.s2Signal = this.candi.getS2Closed();
    }

    private void configureMotor() {

        // Why are we in coast here?
        this.hoodMotor.setNeutralMode(NeutralModeValue.Coast);

        final Slot0Configs pid = new Slot0Configs()
                .withKP(HoodConstants.kP)
                .withKI(HoodConstants.kI)
                .withKD(HoodConstants.kD)
                .withKS(HoodConstants.kS)
                .withKV(HoodConstants.kV)
                .withKA(HoodConstants.kA);

        final SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold((HoodConstants.MAX_HOOD_ANGLE.in(Degrees))) // rotations
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(HoodConstants.MIN_HOOD_ANGLE.in(Degrees)); // rotations

        final FeedbackConfigs feedbackUnits = new FeedbackConfigs()
                .withSensorToMechanismRatio(HoodConstants.SENSOR_TO_MECHANISM_RATIO / 360);

        final TorqueCurrentConfigs torqueDeadband = new TorqueCurrentConfigs()
                .withTorqueNeutralDeadband(1.2);

        this.hoodMotor.getConfigurator().apply(pid);
        this.hoodMotor.getConfigurator().apply(softLimits);
        this.hoodMotor.getConfigurator().apply(feedbackUnits);
        this.hoodMotor.getConfigurator().apply(torqueDeadband);

        this.zeroHood();
    }

    public void setHoodAngle() {

        final ShotData shot = ShotCache.get();

        double desiredAngle = shot.getHoodAngle().in(Degrees);
        desiredAngle *= 0.99;

        this.hoodMotor.setControl(this.motionMagic.withPosition(desiredAngle));
    }

    public void lowerHood() {
        this.hoodMotor.setControl(this.motionMagic.withPosition(HoodConstants.MAX_HOOD_ANGLE.in(Degrees)));
    }

    public void lowerHoodFree() {
        this.hoodMotor.setControl(this.motionMagicFree.withPosition(HoodConstants.ZERO_HOOD_ANGLE.in(Degrees)));

    }

    // public void manualReset() {
    // SmartDashboard.put
    // hoodMotor.setPosition(HoodConstants.MAX_HOOD_ANGLE.in(Degrees));
    // }

    // public double getHoodDegrees(){
    // return hoodPosition.getValueAsDouble();
    // }

    public void zeroHood() {
        this.hoodMotor.setPosition(HoodConstants.STOW_POSITION);
    }

    public boolean isSwitchPressed() {
        return !this.s2Signal.getValue();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(this.s2Signal);

        // SmartDashboard.putNumber("Real Hood Angle", getHoodDegrees());

        final boolean pressed = this.isSwitchPressed();
        if (pressed && !this.wasPressed) {
            this.zeroHood();
        }
        this.wasPressed = pressed;
    }
}
