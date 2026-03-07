package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeArmConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeArm extends SubsystemBase {

    private final TalonFX intakeArmMotor = new TalonFX(
        Constants.CanIdCanivore.Intake_Arm,
        Constants.CanIdCanivore.canivore
    );

    private final CANdi intakeCANdi = new CANdi(
        Constants.CanIdCanivore.Intake_CANdi,
        Constants.CanIdCanivore.canivore
    );

    private final StatusSignal<Current> current = intakeArmMotor.getStatorCurrent();
    private final StatusSignal<Boolean> isSwitchNotPressed = intakeCANdi.getS1Closed();

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

        FeedbackConfigs feedbackUnits = new FeedbackConfigs()
            .withSensorToMechanismRatio(IntakeArmConstants.sensorToMechanismRatio);

        Slot0Configs pid = new Slot0Configs()
            .withKP(IntakeArmConstants.kP)
            .withKI(IntakeArmConstants.kI)
            .withKD(IntakeArmConstants.kD)
            .withKS(IntakeArmConstants.kS)
            .withKV(IntakeArmConstants.kV)
            .withKA(IntakeArmConstants.kA)
            .withKG(IntakeArmConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive);
        
        TorqueCurrentConfigs torqueDeadband = new TorqueCurrentConfigs()
            .withTorqueNeutralDeadband(6.5);

        intakeArmMotor.getConfigurator().apply(pid);
        intakeArmMotor.getConfigurator().apply(feedbackUnits);
        intakeArmMotor.getConfigurator().apply(motorOutput);
        intakeArmMotor.getConfigurator().apply(torqueDeadband);
    }

    public void setIntakePosition(double pos){
        intakeArmMotor.setControl(  
            motionMagic.withPosition(pos));
    }

    public double getIntakePosition() {
        return intakeArmMotor.getPosition().getValueAsDouble();
    }

    public double getCurrent(){
        return current.getValueAsDouble();
    }

    public void zeroIntake() {
        intakeArmMotor.setPosition(IntakeArmConstants.stowPosition);
    }

    public boolean isSwitchPressed() {
        return !isSwitchNotPressed.getValue();
    }

    public void setBrakeMode() {
        intakeArmMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setCoastMode() {
        intakeArmMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {

        BaseStatusSignal.refreshAll(current, isSwitchNotPressed);

        if(isSwitchPressed()) {
            zeroIntake();
        }
    }
}