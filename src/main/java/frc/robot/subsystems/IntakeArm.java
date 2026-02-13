package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeArm extends SubsystemBase{

    CANBus canivore = new CANBus(Constants.CanIdCanivore.ID);

    private final TalonFX intakeRollerMotor = new TalonFX(Constants.CanIdCanivore.Intake_Arm, canivore);
      // Motion Magic controller object
    private final MotionMagicVoltage motionMagic =
        new MotionMagicVoltage(
            TurretConstants.stowPosition//, TODO: change to DynamicMotionMagicTorqueCurrentFOC
            //TurretConstants.velocity,
            //TurretConstants.acceleration
        );

        
    public IntakeArm(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
            .withKP(IntakeArmConstants.kP)
            .withKI(IntakeArmConstants.kI)
            .withKD(IntakeArmConstants.kD)
            .withKS(IntakeArmConstants.kS)
            .withKV(IntakeArmConstants.kV)
            .withKA(IntakeArmConstants.kA);

    //  MotionMagicConfigs motionMagic = new MotionMagicConfigs()
    //         .withMotionMagicCruiseVelocity(TurretConstants.velocity) 
    //         .withMotionMagicAcceleration(TurretConstants.acceleration);

        intakeRollerMotor.getConfigurator().apply(pid);
    //  turretMotor.getConfigurator().apply(motionMagic);
    }

    public void setIntakePosition(double velo){
        intakeRollerMotor.setControl(  
            motionMagic.withPosition(velo));
    }

    public double getIntakePosition() {
        return intakeRollerMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     m_shooter.end(true);
        // }
        SmartDashboard.putNumber("Intake Position", getIntakePosition());
    }
}