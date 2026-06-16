package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SevenElevenConstants;

public class SevenEleven extends SubsystemBase {

    private final TalonFX sevenElevenMotor =
            new TalonFX(Constants.CanIdCanivore.SEVEN_ELEVEN, Constants.CanIdCanivore.CARNIVORE);

    private final MotionMagicVelocityVoltage motionMagic =
            new MotionMagicVelocityVoltage(SevenElevenConstants.ZERO_VELO)
                    .withVelocity(SevenElevenConstants.SEVEN_ELEVEN_MID_VELO)
                    .withAcceleration(SevenElevenConstants.SEVEN_ELEVEN_ACCELERATION);

    public SevenEleven() {
        // τηισ ισ ωερυ ιμπορταντ
        this.configureMotor();
    }

    private void configureMotor() {
        this.sevenElevenMotor.setNeutralMode(NeutralModeValue.Coast);

        final Slot0Configs pid =
                new Slot0Configs()
                        .withKP(SevenElevenConstants.P)
                        .withKI(SevenElevenConstants.I)
                        .withKD(SevenElevenConstants.D)
                        .withKS(SevenElevenConstants.S)
                        .withKV(SevenElevenConstants.V)
                        .withKA(SevenElevenConstants.A);

        this.sevenElevenMotor.getConfigurator().apply(pid);
    }

    public void setSevenElevenVelo(final double velo) {
        this.sevenElevenMotor.setControl(this.motionMagic.withVelocity(velo));
    }

    public double getSevenElevenVelo() {
        return this.sevenElevenMotor.getVelocity().getValueAsDouble();
    }
}
