package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.configs.Slot0Configs;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.utils.Target;

public class Hood extends SubsystemBase{

    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    private final TalonFX hoodMotor = new TalonFX(
        Constants.CanIdCanivore.Hood,
        Constants.CanIdCanivore.canivore);
    
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

        hoodMotor.getConfigurator().apply(pid);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            s1Signal,
            hoodPosition
        );
    }
    
    
    public void setHoodAngle() {

        ShotData shot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                swerve.getState().Pose, 
                new ChassisSpeeds(), 
                Target.getTarget(), 
                3
        );

        double desiredAngle = shot.getHoodAngle().magnitude();

        hoodMotor.setControl(motionMagic.withPosition(Rotations.of(desiredAngle)));
    }

    public double getHoodDegrees(){
        
        return Math.toDegrees(hoodPosition.getValueAsDouble());

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
        if(isSwitchPressed()){
            zeroHood();
        }

        SmartDashboard.putNumber("Hood Position", getHoodDegrees());
    }

}
