package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.ShotCache;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        FeedbackConfigs feedback = new FeedbackConfigs()
            .withSensorToMechanismRatio(ShooterConstants.metersPerRotation);

        shooterMotor.getConfigurator().apply(pid);
        shooterMotor.getConfigurator().apply(motorOutput);
        shooterMotor.getConfigurator().apply(feedback);
    }

    public void calculateShot() {

        ShotData shot = ShotCache.get();

        double Velo = shot.getExitVelocity().in(MetersPerSecond);

        shooterMotor.setControl(motionMagic.withVelocity(Velo));
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