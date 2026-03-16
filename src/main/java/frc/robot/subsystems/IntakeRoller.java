package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;

import static frc.robot.Constants.IntakeRollerConstants.intakeVelo;

import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeRoller extends SubsystemBase{

    private final TalonFX intakeRollerMotor = new TalonFX(
        Constants.CanIdCanivore.Intake_Roller,
        Constants.CanIdCanivore.canivore
    );
    // Motion Magic controller object

    private final MotionMagicVelocityVoltage motionMagic =
        new MotionMagicVelocityVoltage(IntakeRollerConstants.zeroVelo)
           .withVelocity(IntakeRollerConstants.intakeVelo)
           .withAcceleration(IntakeRollerConstants.intakeAcc
        );

    public IntakeRoller(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
            .withKP(IntakeRollerConstants.kP)
            .withKI(IntakeRollerConstants.kI)
            .withKD(IntakeRollerConstants.kD)
            .withKS(IntakeRollerConstants.kS)
            .withKV(IntakeRollerConstants.kV)
            .withKA(IntakeRollerConstants.kA);

        intakeRollerMotor.getConfigurator().apply(pid);
    }

    public void setIntakeVelo(double velo){
        intakeRollerMotor.setControl(  
            motionMagic.withVelocity(velo));
    }

    public boolean isIntaking() {
        return intakeRollerMotor.getVelocity().getValueAsDouble() > intakeVelo * 0.3;
    }

    public double getIntakeVelo() {
        return intakeRollerMotor.getVelocity().getValueAsDouble();
    }
}