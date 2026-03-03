package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeArm extends SubsystemBase{

    private final TalonFX intakeArmMotor = new TalonFX(
        Constants.CanIdCanivore.Intake_Arm,
        Constants.CanIdCanivore.canivore
    );

    private final CANdi intakeCANdi = new CANdi(
        Constants.CanIdCanivore.Intake_CANdi,
        Constants.CanIdCanivore.canivore
    );

    private StatusSignal<Boolean> s1Signal = intakeCANdi.getS1Closed();

    // Motion Magic controller object
    private final DynamicMotionMagicTorqueCurrentFOC motionMagic =
        new DynamicMotionMagicTorqueCurrentFOC(
            IntakeArmConstants.stowPosition,
            IntakeArmConstants.velocity,
            IntakeArmConstants.acceleration
        );

        
    public IntakeArm(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        intakeArmMotor.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs pid = new Slot0Configs()
            .withKP(IntakeArmConstants.kP)
            .withKI(IntakeArmConstants.kI)
            .withKD(IntakeArmConstants.kD)
            .withKS(IntakeArmConstants.kS)
            .withKV(IntakeArmConstants.kV)
            .withKA(IntakeArmConstants.kA);

        intakeArmMotor.getConfigurator().apply(pid);
    }

    public void setIntakePosition(double pos){
        intakeArmMotor.setControl(  
            motionMagic.withPosition(pos));
    }

    public double getIntakePosition() {
        return intakeArmMotor.getPosition().getValueAsDouble();
    }

    public void zeroIntake() {
        intakeArmMotor.setPosition(0);
    }

    public boolean isSwitchPressed(){
        return !s1Signal.getValue();
    }

    @Override
    public void periodic() {
        
        if(!s1Signal.getValue()) {
            zeroIntake();
        }
    }
}