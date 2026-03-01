package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClimbConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climb extends SubsystemBase{

    private final TalonFX climbMotor = new TalonFX(
        Constants.CanIdCanivore.Climb,
        Constants.CanIdCanivore.canivore);

    // Motion Magic controller object
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic =
        new DynamicMotionMagicTorqueCurrentFOC(
            ClimbConstants.downPosition,
            ClimbConstants.velocity,
            ClimbConstants.acceleration
        );

    public Climb(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        climbMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
            .withKP(ClimbConstants.kP)
            .withKI(ClimbConstants.kI)
            .withKD(ClimbConstants.kD)
            .withKS(ClimbConstants.kS)
            .withKV(ClimbConstants.kV)
            .withKA(ClimbConstants.kA)
            .withKG(ClimbConstants.kG);

        climbMotor.getConfigurator().apply(pid);
    }

    public void setClimbPosition(double pos){
        climbMotor.setControl(  
            motionMagic.withPosition(pos));
    }

    public double getClimbPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }
}