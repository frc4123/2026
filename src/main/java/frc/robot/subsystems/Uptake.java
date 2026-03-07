package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.UptakeConstants;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Uptake extends SubsystemBase{

    private final TalonFX uptakeMotor = new TalonFX(
        Constants.CanIdCanivore.Uptake,
        Constants.CanIdCanivore.canivore
    );

    private final MotionMagicVelocityVoltage motionMagic =
        new MotionMagicVelocityVoltage(UptakeConstants.zeroVelo)
           .withVelocity(UptakeConstants.uptakeVelo)
           .withAcceleration(UptakeConstants.uptakeAcc
        );

    public Uptake(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        uptakeMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
            .withKP(UptakeConstants.kP)
            .withKI(UptakeConstants.kI)
            .withKD(UptakeConstants.kD)
            .withKS(UptakeConstants.kS)
            .withKV(UptakeConstants.kV)
            .withKA(UptakeConstants.kA);

        uptakeMotor.getConfigurator().apply(pid);
    }

    public void setUptakeVelo(double velo){
        uptakeMotor.setControl(  
            motionMagic.withVelocity(velo));
    }

    public double getUptakeVelo() {
        return uptakeMotor.getVelocity().getValueAsDouble();
    }


    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     m_shooter.end(true);
        // }
        //SmartDashboard.putNumber("Uptake Velo", getUptakeVelo());
    }
}