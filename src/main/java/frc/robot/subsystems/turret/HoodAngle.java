package frc.robot.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;

import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class HoodAngle {

    private final TalonFX hoodMotor =
        new TalonFX(Constants.CanIdCanivore.Hood,
                    Constants.CanIdCanivore.ID);

    private final MotionMagicVoltage motionMagicRequest =
        new MotionMagicVoltage(0);

    public HoodAngle() {
        configureMotor();
    }

   
    private void configureMotor() {

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

      
        Slot0Configs pid = new Slot0Configs()
            .withKP(HoodConstants.kP)
            .withKI(HoodConstants.kI)
            .withKD(HoodConstants.kD)
            .withKS(HoodConstants.kS)
            .withKV(HoodConstants.kV)
            .withKA(HoodConstants.kA);

   
        MotionMagicConfigs motionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.HoodConstants.velocity)
            .withMotionMagicAcceleration(Constants.HoodConstants.acceleration);

        hoodMotor.getConfigurator().apply(pid);
        hoodMotor.getConfigurator().apply(motionMagic);
    }
    public void setHoodAngle(){
        

    }


}
