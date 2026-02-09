package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CanIdCanivore;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
    }

    public void setShooterVelo(double velo){
        intakeRollerMotor.setControl(motionMagic.)
    }

    public double getIntakeVelo() {
        return 
    }


    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     m_shooter.end(true);
        // }
        SmartDashboard.putNumber("Intake Velo", getIntakeVelo());
    }
}