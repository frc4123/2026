package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArm extends SubsystemBase {

    private final TalonFX intakeArmMotor = new TalonFX(
            Constants.CanIdCanivore.INTAKE_ARM,
            Constants.CanIdCanivore.CARNIVORE);

    private final CANdi candi;

    private final StatusSignal<Boolean> isSwitchNotPressed;

    private boolean hasZeroed = false;

    private final DynamicMotionMagicTorqueCurrentFOC motionMagic = new DynamicMotionMagicTorqueCurrentFOC(
            IntakeArmConstants.STOW_POSITION,
            IntakeArmConstants.VELOCITY,
            IntakeArmConstants.ACCELERATION);

    private final DynamicMotionMagicTorqueCurrentFOC slowMotionMagic = new DynamicMotionMagicTorqueCurrentFOC(
            IntakeArmConstants.STOW_POSITION,
            IntakeArmConstants.SLOW_VELOCITY,
            IntakeArmConstants.ACCELERATION);

    private final DynamicMotionMagicTorqueCurrentFOC midMotionMagic = new DynamicMotionMagicTorqueCurrentFOC(
            IntakeArmConstants.STOW_POSITION,
            IntakeArmConstants.MID_VELOCITY,
            IntakeArmConstants.ACCELERATION);

    public IntakeArm(final CANdi candi) {
        this.candi = candi;
        this.configureMotor();
        this.isSwitchNotPressed = this.candi.getS1Closed();
    }

    private void configureMotor() {
        this.intakeArmMotor.setNeutralMode(NeutralModeValue.Brake);

        final FeedbackConfigs feedbackUnits = new FeedbackConfigs()
                .withSensorToMechanismRatio(IntakeArmConstants.SENSOR_TO_MECHANISM_RATIO);

        final Slot0Configs pid = new Slot0Configs()
                .withKP(IntakeArmConstants.P)
                .withKI(IntakeArmConstants.I)
                .withKD(IntakeArmConstants.D)
                .withKS(IntakeArmConstants.S)
                .withKV(IntakeArmConstants.V)
                .withKA(IntakeArmConstants.A)
                .withKG(IntakeArmConstants.G)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        final MotorOutputConfigs motorOutput = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);

        final TorqueCurrentConfigs torqueDeadband = new TorqueCurrentConfigs()
                .withTorqueNeutralDeadband(6.5);

        this.intakeArmMotor.getConfigurator().apply(pid);
        this.intakeArmMotor.getConfigurator().apply(feedbackUnits);
        this.intakeArmMotor.getConfigurator().apply(motorOutput);
        this.intakeArmMotor.getConfigurator().apply(torqueDeadband);
    }

    public void setIntakePosition(final double pos) {
        this.intakeArmMotor.setControl(
                this.motionMagic.withPosition(pos));
    }

    public void setSlowIntakePosition(final double pos) {
        this.intakeArmMotor.setControl(
                this.slowMotionMagic.withPosition(pos));
    }

    public void setMidIntakePosition(final double pos) {
        this.intakeArmMotor.setControl(
                this.midMotionMagic.withPosition(pos));
    }

    public double getIntakePosition() {
        return this.intakeArmMotor.getPosition().getValueAsDouble();
    }

    public void zeroIntake() {
        this.intakeArmMotor.setPosition(IntakeArmConstants.STOW_POSITION);
    }

    public boolean isSwitchPressed() {
        return !this.isSwitchNotPressed.getValue();
    }

    public void setBrakeMode() {
        this.intakeArmMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        this.intakeArmMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(this.isSwitchNotPressed);

        if (this.isSwitchPressed() && !this.hasZeroed) {
            this.zeroIntake();
            this.hasZeroed = true;
        }

        if (!this.isSwitchPressed()) {
            this.hasZeroed = false;
        }
    }
}