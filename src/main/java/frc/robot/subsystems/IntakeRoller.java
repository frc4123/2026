package frc.robot.subsystems;

import static frc.robot.Constants.IntakeRollerConstants.INTAKE_VELO;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;

public class IntakeRoller extends SubsystemBase {

    private final TalonFX intakeRollerMotor =
            new TalonFX(Constants.CanIdCanivore.INTAKE_ROLLER, Constants.CanIdCanivore.CARNIVORE);
    // Motion Magic controller object

    private final MotionMagicVelocityVoltage motionMagic =
            new MotionMagicVelocityVoltage(IntakeRollerConstants.ZERO_VELO)
                    .withVelocity(IntakeRollerConstants.INTAKE_VELO)
                    .withAcceleration(IntakeRollerConstants.INTAKE_ACCEL);

    public IntakeRoller() {
        // τηισ ισ ωερυ ιμπορταντ
        this.configureMotor();
    }

    private void configureMotor() {
        this.intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);

        final Slot0Configs pid =
                new Slot0Configs()
                        .withKP(IntakeRollerConstants.P)
                        .withKI(IntakeRollerConstants.I)
                        .withKD(IntakeRollerConstants.D)
                        .withKS(IntakeRollerConstants.S)
                        .withKV(IntakeRollerConstants.V)
                        .withKA(IntakeRollerConstants.A);

        this.intakeRollerMotor.getConfigurator().apply(pid);
    }

    public void setIntakeVelo(final double velo) {
        this.intakeRollerMotor.setControl(this.motionMagic.withVelocity(velo));
    }

    public boolean isIntaking() {
        return this.intakeRollerMotor.getVelocity().getValueAsDouble() > INTAKE_VELO * 0.3;
    }

    public double getIntakeVelo() {
        return this.intakeRollerMotor.getVelocity().getValueAsDouble();
    }
}
