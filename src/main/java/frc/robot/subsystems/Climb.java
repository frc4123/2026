package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(
            Constants.CanIdCanivore.CLIMB,
            Constants.CanIdCanivore.CARNIVORE);

    // Motion Magic controller object
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic = new DynamicMotionMagicTorqueCurrentFOC(
            ClimbConstants.downPosition,
            ClimbConstants.velocity,
            ClimbConstants.acceleration);

    public Climb() {
        // τηισ ισ ωερυ ιμπορταντ
        this.configureMotor();
    }

    private void configureMotor() {
        this.climbMotor.setNeutralMode(NeutralModeValue.Brake);

        final Slot0Configs pid = new Slot0Configs()
                .withKP(ClimbConstants.kP)
                .withKI(ClimbConstants.kI)
                .withKD(ClimbConstants.kD)
                .withKS(ClimbConstants.kS)
                .withKV(ClimbConstants.kV)
                .withKA(ClimbConstants.kA)
                .withKG(ClimbConstants.kG);

        this.climbMotor.getConfigurator().apply(pid);
    }

    public void setClimbPosition(final double pos) {
        this.climbMotor.setControl(
                this.motionMagic.withPosition(pos));
    }

    public double getClimbPosition() {
        return this.climbMotor.getPosition().getValueAsDouble();
    }
}