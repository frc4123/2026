package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanIdCanivore;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeRollers extends SubsystemBase{

    CANBus canivore = new CANBus(Constants.CanIdCanivore.ID);

    private final TalonFX intakeRollerMotor = new TalonFX(Constants.CanIdCanivore.Turret, canivore);
      // Motion Magic controller object
    private final MotionMagicVelocityVoltage motionMagic =
        new MotionMagicVelocityVoltage(
            TurretConstants.stowPosition//, TODO: change to DynamicMotionMagicTorqueCurrentFOC
            //TurretConstants.velocity,
            //TurretConstants.acceleration
        );

        
    public IntakeRollers(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        intakeRollerMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
                .withKP(IntakeConstants.kP)
                .withKI(IntakeConstants.kI)
                .withKD(IntakeConstants.kD)
                .withKS(IntakeConstants.kS)
                .withKV(IntakeConstants.kV)
                .withKA(IntakeConstants.kA);

    //  MotionMagicConfigs motionMagic = new MotionMagicConfigs()
    //         .withMotionMagicCruiseVelocity(TurretConstants.velocity)
    //         .withMotionMagicAcceleration(TurretConstants.acceleration);

        intakeRollerMotor.getConfigurator().apply(pid);
    //  turretMotor.getConfigurator().apply(motionMagic);
    }

    public void setIntakeVelo(double velo){
        intakeRollerMotor.setControl(  
            motionMagic.withVelocity(velo));
    }

    public double getIntakeVelo() {
        return intakeRollerMotor.getVelocity().getValueAsDouble();
    }


    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     m_shooter.end(true);
        // }
        SmartDashboard.putNumber("Intake Velo", getIntakeVelo());
    }
}