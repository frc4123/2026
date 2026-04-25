package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;
import frc.robot.utils.ShotCache;

public class Uptake extends SubsystemBase {

    private final TalonFX uptakeMotor =
            new TalonFX(Constants.CanIdCanivore.UPTAKE, Constants.CanIdCanivore.CARNIVORE);

    private final MotionMagicVelocityVoltage motionMagic =
            new MotionMagicVelocityVoltage(UptakeConstants.ZERO_VELO)
                    .withVelocity(UptakeConstants.UPTAKE_VELO)
                    .withAcceleration(UptakeConstants.UPTAKE_ACCELERATION);

    public Uptake() {
        this.configureMotor();
    }

    private void configureMotor() {
        this.uptakeMotor.setNeutralMode(NeutralModeValue.Brake);

        final Slot0Configs pid =
                new Slot0Configs()
                        .withKP(UptakeConstants.P)
                        .withKI(UptakeConstants.I)
                        .withKD(UptakeConstants.D)
                        .withKS(UptakeConstants.S)
                        .withKV(UptakeConstants.V)
                        .withKA(UptakeConstants.A);

        this.uptakeMotor.getConfigurator().apply(pid);
    }

    public void zeroUptakeVelo() {
        this.uptakeMotor.setControl(this.motionMagic.withVelocity(0));
    }

    public void setUptakeVelo() {
        final ShotData shot = ShotCache.get();

        final double Velo =
                shot.getExitVelocity().in(MetersPerSecond)
                        * (2)
                        / (2.0
                                * Math.PI
                                * (ShooterConstants.FLYWHEEL_RADIUS.in(Meters)
                                        + ShooterConstants.compression.in(Meters)));

        // THIS IS THE RATIO I DETERMIEND TO SHOOT FARTHER IF NEEDED IF IT MISSES SHOO
        // ShooterConstants.shootingTestErrorRatio; so multiply the final velo by that

        this.uptakeMotor.setControl(
                this.motionMagic.withVelocity(
                        (Velo * ShooterConstants.SHOOTING_TEST_ERROR_RATIO * 0.8))); // 1.23
    }

    public double getUptakeVelo() {
        return this.uptakeMotor.getVelocity().getValueAsDouble();
    }
}
