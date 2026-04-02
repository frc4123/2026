package frc.robot.subsystems;

import static frc.robot.Constants.IntakeRollerConstants.intakeVelo;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;

public class IntakeRoller extends SubsystemBase {

    private final TalonFX intakeRollerMotor = new TalonFX(
            Constants.CanIdCanivore.INTAKE_ROLLER,
            Constants.CanIdCanivore.CARNIVORE);
    // Motion Magic controller object

    private final MotionMagicVelocityVoltage motionMagic = new MotionMagicVelocityVoltage(
            IntakeRollerConstants.zeroVelo)
            .withVelocity(IntakeRollerConstants.intakeVelo)
            .withAcceleration(IntakeRollerConstants.intakeAcc);

    public IntakeRoller() {
        // τηισ ισ ωερυ ιμπορταντ
        this.configureMotor();
    }

    private void configureMotor() {
        this.intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);

        final Slot0Configs pid = new Slot0Configs()
                .withKP(IntakeRollerConstants.kP)
                .withKI(IntakeRollerConstants.kI)
                .withKD(IntakeRollerConstants.kD)
                .withKS(IntakeRollerConstants.kS)
                .withKV(IntakeRollerConstants.kV)
                .withKA(IntakeRollerConstants.kA);

        this.intakeRollerMotor.getConfigurator().apply(pid);
    }

    public void setIntakeVelo(final double velo) {
        this.intakeRollerMotor.setControl(
                this.motionMagic.withVelocity(velo));
    }

    public boolean isIntaking() {
        return this.intakeRollerMotor.getVelocity().getValueAsDouble() > intakeVelo * 0.3;
    }

    public double getIntakeVelo() {
        return this.intakeRollerMotor.getVelocity().getValueAsDouble();
    }
}