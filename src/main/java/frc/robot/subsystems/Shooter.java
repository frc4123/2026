package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.ShotCache;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase{

    private final TalonFX shooterMotor = new TalonFX(
        Constants.CanIdCanivore.Shooter,
        Constants.CanIdCanivore.canivore
    );

    private final VelocityTorqueCurrentFOC motionMagic =
        new VelocityTorqueCurrentFOC(ShooterConstants.MIN_SPEED.in(MetersPerSecond))
            .withAcceleration(ShooterConstants.acceleration
        );

    private final VelocityTorqueCurrentFOC slowMotionMagic =
        new VelocityTorqueCurrentFOC(ShooterConstants.MIN_SPEED.in(MetersPerSecond))
            .withAcceleration(ShooterConstants.slowAcceleration
        );

    public boolean isShooting = false;
        
    public Shooter(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        Slot0Configs pid = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD)
            .withKS(ShooterConstants.kS)
            .withKV(ShooterConstants.kV)
            .withKA(ShooterConstants.kA);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive);
        
        shooterMotor.getConfigurator().apply(pid);

        shooterMotor.getConfigurator().apply(motorOutput);
    }

    public void calculateShot() {

        ShotData shot = ShotCache.get();

        double Velo = shot.getExitVelocity().in(MetersPerSecond) * (2)
            / (2.0 * Math.PI * (ShooterConstants.flywheelRadius.in(Meters) + ShooterConstants.compression.in(Meters)));

            // THIS IS THE RATIO I DETERMIEND TO SHOOT FARTHER IF NEEDED IF IT MISSES SHOO
            // ShooterConstants.shootingTestErrorRatio; so multiply the final velo by that 

        if(isShooting){
            shooterMotor.setControl(motionMagic.withVelocity((Velo * ShooterConstants.shootingTestErrorRatio)));  //1.23
        } else {
            shooterMotor.setControl(slowMotionMagic.withVelocity((Velo * ShooterConstants.shootingTestErrorRatio)));  //1.23
        }
        
    }

    public void isShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    public void shooterMinVelo() {
        shooterMotor.setControl(motionMagic.withVelocity(ShooterConstants.MIN_SPEED.in(MetersPerSecond)));
    }

    public void setShooterOpenLoopVelo(double velo) {
        shooterMotor.set(velo);
    }

    public double getShooterVelo() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }
}