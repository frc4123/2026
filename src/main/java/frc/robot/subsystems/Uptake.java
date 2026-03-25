package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.ShotCache;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

    public void zeroUptakeVelo(){
        uptakeMotor.setControl(motionMagic.withVelocity(0));
    }

    public void setUptakeVelo(double velo){
        ShotData shot = ShotCache.get();

        double Velo = shot.getExitVelocity().in(MetersPerSecond) * (2)
            / (2.0 * Math.PI * (ShooterConstants.flywheelRadius.in(Meters) + ShooterConstants.compression.in(Meters)));

            // THIS IS THE RATIO I DETERMIEND TO SHOOT FARTHER IF NEEDED IF IT MISSES SHOO
            // ShooterConstants.shootingTestErrorRatio; so multiply the final velo by that 

        uptakeMotor.setControl(motionMagic.withVelocity((Velo * ShooterConstants.shootingTestErrorRatio * 0.8)));  //1.23
    }

    public double getUptakeVelo() {
        return uptakeMotor.getVelocity().getValueAsDouble();
    }
}