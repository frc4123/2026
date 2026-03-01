package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.SevenElevenConstants;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SevenEleven extends SubsystemBase{

    private final TalonFX sevenElevenMotor = new TalonFX(
        Constants.CanIdCanivore.SevenEleven,
        Constants.CanIdCanivore.canivore
    );

    private final MotionMagicVelocityVoltage motionMagic =
        new MotionMagicVelocityVoltage(SevenElevenConstants.zeroVelo)
           .withVelocity(SevenElevenConstants.sevenElevenVelo)
           .withAcceleration(SevenElevenConstants.sevenElevenAcc
        );

    public SevenEleven(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        sevenElevenMotor.setNeutralMode(NeutralModeValue.Coast);

        Slot0Configs pid = new Slot0Configs()
            .withKP(SevenElevenConstants.kP)
            .withKI(SevenElevenConstants.kI)
            .withKD(SevenElevenConstants.kD)
            .withKS(SevenElevenConstants.kS)
            .withKV(SevenElevenConstants.kV)
            .withKA(SevenElevenConstants.kA);

        sevenElevenMotor.getConfigurator().apply(pid);
    }

    public void setSevenElevenVelo(double velo){
        sevenElevenMotor.setControl(  
            motionMagic.withVelocity(velo));
    }

    public double getSevenElevenVelo() {
        return sevenElevenMotor.getVelocity().getValueAsDouble();
    }
}