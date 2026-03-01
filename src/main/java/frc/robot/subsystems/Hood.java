package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.ShotCache;

public class Hood extends SubsystemBase{

    private boolean wasPressed = false;

    private final TalonFX hoodMotor = new TalonFX(
        Constants.CanIdCanivore.Hood,
        Constants.CanIdCanivore.canivore
    );
    
    private final CANdi hoodCANdi = new CANdi(Constants.CanIdCanivore.Hood_CANdi, Constants.CanIdCanivore.canivore);

    private StatusSignal<Boolean> s1Signal = hoodCANdi.getS1Closed();
    private StatusSignal<Angle> hoodPosition = hoodMotor.getPosition();


    private final DynamicMotionMagicTorqueCurrentFOC motionMagic =
        new DynamicMotionMagicTorqueCurrentFOC(
            HoodConstants.stowPosition,
            HoodConstants.velocity,
            HoodConstants.acceleration
            
        );

    public Hood() {
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

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold((HoodConstants.MAX_HOOD_ANGLE.in(Degrees)))  // rotations
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(HoodConstants.MIN_HOOD_ANGLE.in(Degrees));  // rotations

        FeedbackConfigs feedbackUnits = new FeedbackConfigs()
            .withSensorToMechanismRatio(HoodConstants.sensorToMechanismRatio / 360);

        hoodMotor.getConfigurator().apply(pid);
        hoodMotor.getConfigurator().apply(softLimits);
        hoodMotor.getConfigurator().apply(feedbackUnits);

        zeroHood();   
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            s1Signal,
            hoodPosition
        );
    }
    
    public void setHoodAngle() {

        ShotData shot = ShotCache.get();

        double desiredAngle = shot.getHoodAngle().in(Degrees);

        hoodMotor.setControl(motionMagic.withPosition(desiredAngle));
    }

    public double getHoodDegrees(){
        return hoodPosition.getValueAsDouble();
    }

    public void zeroHood(){
        hoodMotor.setPosition(HoodConstants.stowPosition);
    }

    public boolean isSwitchPressed(){
        return !s1Signal.getValue();
    }

    @Override 
    public void periodic(){
        refreshStatusSignals();

        boolean pressed = isSwitchPressed();
        if (pressed && !wasPressed) {
            zeroHood();
        }
        wasPressed = pressed;
    }
}
