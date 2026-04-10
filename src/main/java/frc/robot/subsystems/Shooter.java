package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.ShotCache;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterMotor =
            new TalonFX(Constants.CanIdCanivore.SHOOTER, Constants.CanIdCanivore.CARNIVORE);

    private final VelocityTorqueCurrentFOC motionMagic =
            new VelocityTorqueCurrentFOC(ShooterConstants.MIN_SPEED.in(MetersPerSecond))
                    .withAcceleration(ShooterConstants.ACCELERATION);

    private final VelocityTorqueCurrentFOC slowMotionMagic =
            new VelocityTorqueCurrentFOC(ShooterConstants.MIN_SPEED.in(MetersPerSecond))
                    .withAcceleration(ShooterConstants.SLOW_ACCELERATION);

    private boolean isShooting = false;

    private static String shootingString = "Shooting Slider";

    private double onTheGoSlider = 1.0;

    public Shooter() {
        this.configureMotor();
        SmartDashboard.putNumber(Shooter.shootingString, 1.0);
    }

    private void configureMotor() {
        this.shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        final Slot0Configs pid =
                new Slot0Configs()
                        .withKP(ShooterConstants.P)
                        .withKI(ShooterConstants.I)
                        .withKD(ShooterConstants.D)
                        .withKS(ShooterConstants.S)
                        .withKV(ShooterConstants.V)
                        .withKA(ShooterConstants.A);

        final MotorOutputConfigs motorOutput =
                new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive);

        this.shooterMotor.getConfigurator().apply(pid);

        this.shooterMotor.getConfigurator().apply(motorOutput);
    }

    public void calculateShot() {

        final ShotData shot = ShotCache.get();

        final double Velo =
                shot.getExitVelocity().in(MetersPerSecond)
                        * (2)
                        / (2.0
                                * Math.PI
                                * (ShooterConstants.FLYWHEEL_RADIUS.in(Meters)
                                        + ShooterConstants.compression.in(Meters)))
                        * this.onTheGoSlider
                        * .97;

        // THIS IS THE RATIO I DETERMIEND TO SHOOT FARTHER IF NEEDED IF IT MISSES SHOO
        // ShooterConstants.shootingTestErrorRatio; so multiply the final velo by that

        if (this.isShooting) {
            this.shooterMotor.setControl(
                    this.motionMagic.withVelocity(
                            (Velo * ShooterConstants.SHOOTING_TEST_ERROR_RATIO))); // 1.23
        } else {
            this.shooterMotor.setControl(
                    this.slowMotionMagic.withVelocity(
                            (Velo * ShooterConstants.SHOOTING_TEST_ERROR_RATIO))); // 1.23
        }
    }

    public void setShooting(final boolean isShooting) {
        this.isShooting = isShooting;
    }

    public void shooterMinVelo() {
        this.shooterMotor.setControl(
                this.motionMagic.withVelocity(ShooterConstants.MIN_SPEED.in(MetersPerSecond)));
    }

    public void setShooterOpenLoopVelo(final double velo) {
        this.shooterMotor.set(velo);
    }

    public double getShooterVelo() {
        return this.shooterMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        this.onTheGoSlider = SmartDashboard.getNumber(Shooter.shootingString, 1.0);
    }
}
