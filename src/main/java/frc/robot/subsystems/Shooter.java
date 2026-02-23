package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.Target;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase{

    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

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

        FeedbackConfigs feedbackUnits = new FeedbackConfigs()
            .withSensorToMechanismRatio(ShooterConstants.sensorToMechanismRatio);

        shooterMotor.getConfigurator().apply(feedbackUnits);
        shooterMotor.getConfigurator().apply(pid);
    }

    public void setShooterVelo() {

        ShotData shot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                swerve.getState().Pose, 
                new ChassisSpeeds(), 
                Target.getTarget(), 
                3
        );

        double Velo = shot.getExitVelocity().in(MetersPerSecond)
            / (2 * Math.PI * ShooterConstants.flywheelRadius.in(Meters));

        shooterMotor.setControl(motionMagic.withVelocity(Velo));
    }

    public double getShooterVelo() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     m_shooter.end(true);
        // }
        SmartDashboard.putNumber("Shooter Velo", getShooterVelo());
    }
}