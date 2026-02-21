package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeArm extends SubsystemBase{

    private final TalonFX intakeMotor = new TalonFX(Constants.CanIdCanivore.Intake_Arm, Constants.CanIdCanivore.canivore);
    private final CANdi intakeCANdi = new CANdi(Constants.CanIdCanivore.Intake_CANdi, Constants.CanIdCanivore.canivore);

    private StatusSignal<Boolean> s1Signal = intakeCANdi.getS1Closed();

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
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

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

        intakeMotor.getConfigurator().apply(pid);
    //  turretMotor.getConfigurator().apply(motionMagic);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            s1Signal
        );
    }

    public void setIntakePosition(double pos){
        intakeMotor.setControl(  
            motionMagic.withPosition(pos));
    }

    public double getIntakePosition() {
        return intakeMotor.getPosition().getValueAsDouble();
    }

    public void zeroIntake() {
        intakeMotor.setPosition(0);
    }

    public boolean isSwitchPressed(){
        return !s1Signal.getValue();
    }

    @Override
    public void periodic() {
        // if (DriverStation.isEnabled()) {
        //     m_shooter.end(true);
        // }
        refreshStatusSignals();
        if(isSwitchPressed()) {
            zeroIntake();
        }

        SmartDashboard.putNumber("Intake Position", getIntakePosition());
    }
}