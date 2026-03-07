package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static final class CanIdCanivore { 

        public static final CANBus canivore = new CANBus("Good Boy CANivore 10");

        public static final int Front_Left_Drive = 2;
        public static final int Front_Right_Drive = 3;
        public static final int Back_Left_Drive = 4;
        public static final int Back_Right_Drive = 5;
        // drive motors - order - start top left in clockwise rotation

        public static final int Front_Left_Turn = 6;
        public static final int Front_Right_Turn = 7;
        public static final int Back_Left_Turn = 8;
        public static final int Back_Right_Turn = 9;
        // turn motors - order - start top left in clockwise rotation

        public static final int Pigeon = 10;

        public static final int Front_Left_CANcoder = 11;
        public static final int Front_Right_CANcoder = 12;
        public static final int Back_Left_CANcoder = 13;
        public static final int Back_Right_CANcoder = 14;

        public static final int Turret = 15;
        public static final int Turret_Encoder1 = 16;
        public static final int Turret_Encoder2 = 17;

        public static final int Intake_CANdi = 18;

        public static final int Intake_Arm = 19;

        public static final int Intake_Roller = 20;

        public static final int Hood = 21;

        public static final int Shooter = 23;

        public static final int Climb = 24;

        public static final int SevenEleven = 25;

        public static final int Uptake = 26;
    }

    public static final class InputConstants {

        public static final int kDriverControllerPort0 = 0;
        public static final int kDriverControllerPort1 = 1;
        public static final int kDriverControllerPort2 = 2;
        public static final boolean fieldOrientation = true;
        public static final double kDeadband = 0.028;
    }

    public static final class SwerveConstants {

        public static final double CLOSE_TRANSLATION_PP_KP = 2.5; // 8
        public static final double CLOSE_TRANSLATION_PP_KI = 0.0;
        public static final double CLOSE_TRANSLATION_PP_KD = 0.0;

        public static final double CLOSE_ROTATION_PP_KP = 5; // 8
        public static final double CLOSE_ROTATION_PP_KI = 0.0;
        public static final double CLOSE_ROTATION_PP_KD = 0.0;

        public static final Pose2d BLUE_CLIMB_POSE = new Pose2d(0.0080772, 3.7457125999999996, new Rotation2d(0 * Math.PI / 180.0)); // id 31
        public static final Pose2d RED_CLIMB_POSE = new Pose2d(16.53296166, 4.3235626, new Rotation2d(0 * Math.PI / 180.0)); // id 15
        
        public static final double[][] ADDITIONS = {
            {1.0549228, 1.0422874}, // LEFT ADDITION // {0.342, 0} //0.385
            {1.0549228, -1.0297126}  // RIGHT ADDITION // {0.342, 0.348} //0.385 was correct in odometry w advantagescope
            // driver relative -> {+forward/back-, +left/right-}
        };

        public static final Pose2d robotToClimber = new Pose2d(Units.inchesToMeters(2), Units.inchesToMeters(27.5/2.0), new Rotation2d());
        //33.75 bumper width
    }
   
    public static class IntakeArmConstants {
       
        public static final double outPosition = 0;
        public static final double stowPosition = 0.33;

        public static final double sensorToMechanismRatio = 25.0;
        public static final double currentCancelationThreshold = 80;
       
        public static final double kP = 5000.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 2.5; 
        public static final double kV = 0; 
        public static final double kA = 0; 
        public static final double kG = 6.14123;

        public static final double velocity = 2.5;
        public static final double acceleration = 2.5;
    }

    public static class IntakeRollerConstants {

        public static final Distance STOW_POS = Inches.of(0);
        public static final Distance DEPLOY_POS = Inches.of(10.875);
        public static final Voltage SPIN_VOLTAGE = Volts.of(3);

        public static final double VEL_MULTIPLIER = 70.0; // multiplies goal velocity for targetting
        public static final double VEL_POWER = 0.3; // raises goal velocity to power
        public static final LinearVelocity BASE_VEL = InchesPerSecond.of(50); // added to final velocity

        public static final double zeroVelo = 0;
        public static final double intakeVelo = 35;
        public static final double intakeAcc = 90;
       
        public static final double kP = 0.426;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 0; 
        public static final double kV = 0.118;  //try this 0;
        public static final double kA = 0; 
    }

    public static class SevenElevenConstants {

        public static final double zeroVelo = 0;
        public static final double sevenElevenVelo = 10;
        public static final double sevenElevenAcc = 20;
       
        public static final double kP = 0.426;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 0; 
        public static final double kV = 0.118;  //try this 0;
        public static final double kA = 0; 
    }

    public static class UptakeConstants {

        public static final double zeroVelo = 0;
        public static final double uptakeVelo = 45;
        public static final double reverseVelo = -45;
        public static final double uptakeAcc = uptakeVelo * 3.0;
       
        public static final double kP = 0.426;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 0; 
        public static final double kV = 0.118;  //try this 0;
        public static final double kA = 0; 
    }

    public static final class TurretConstants {

        public static final double stowPosition = 0;
        public static final double velocity = 5; //2.5 // 1.75 working if bad 1 was working before but its slow.. when your confident that the pid always reaches setpoint then jack ts up
        public static final double acceleration = 5; // 3 before replacing with 1.1 try lowering kv //1.1

        public static final double kP = 0;//300;
        public static final double kI = 0;//11;
        public static final double kD = 0;//10.4123;
        public static final double kS = 9;// 1.25;
        public static final double kV = 0;//0.01; // 0.5 // 4.4 0.64 / 0.15 according to calculation but was innacurate by a little bit irl
        public static final double kA = 0;//0.028; //0.05; // 0.367048

                                    // IN ROTATIONS //
        public static final double mechanismMinRange = -37.0 / 72.0; // -1 is -360 degrees
        public static final double mechanismMaxRange = 37.0 / 72.0; // 1 is +360 degrees
        // this makes total of 720 degrees rotation^^^^

        public static final double mechanismGearTeeth = 85.0; // drives botrh encoders
        public static final double encoder1Teeth = 10.0; //24.0;      // Gear on Hex Shaft A that connects to turret
        public static final double encoder2Teeth = 22.0; // 10.0 / (50.0/22.0); // 22.0;

        public static final double motorToTurretRatio =  (50.0 / 10.0) * (mechanismGearTeeth / encoder1Teeth);// (48.0/9.0) * (180.0/24.0); 

        public static final double rotorToEncoder1Ratio = (50.0 / 10.0); //48.0 / 9.0;
        public static final double sensorToMechanismRatio = (mechanismGearTeeth / encoder1Teeth); //180.0 / 24.0;
    
        //public static final double encoder2Ratio = (turretGearTeeth / encoder1Teeth) * (50.0 / encoder2Teeth);      // Gear on Hex Shaft B that connects to turret

        public static final double encoder1Offset = -0.433105;
        public static final double encoder2Offset = -0.527588;
        //TODO: if the wrap happens to be near the zero measurement (within hundredths check yams for interval confirmation), then RESEAT CANCODERS
                
        public static final double coverageMargin = 1.2;
        public static final int minTeeth = 1;
        public static final int maxTeeth = 1;
        public static final int maxIterations = 30;


        public static final double offsetX = Units.inchesToMeters(8);
        public static final double offsetY = Units.inchesToMeters(-8);
        public static final double offsetZ = Units.inchesToMeters(8);

        public static final double tagOffset = Units.inchesToMeters(47/2);
        public static final double[] validTurretTagsBlue = {21, 26, 18};
        public static final double[] validTurretTagsRed = {2, 10, 5};

        public static final Translation2d turretOffset = new Translation2d(offsetX, offsetY);
        public static final Pose3d robotToTurret = new Pose3d(offsetX, offsetY, offsetZ, new Rotation3d());
        public static final Transform3d transform3D = new Transform3d(robotToTurret, new Pose3d());

        public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20);
    }

    public static final class ShooterConstants {

        public static final LinearVelocity MIN_SPEED = MetersPerSecond.of(6); 

        public static final Distance flywheelRadius = Inches.of(2.03); 

        public static final double metersPerRotation = 2.0 * Math.PI * flywheelRadius.in(Meters);

        public static final double sensorTomechanismGearTeeth = 1.0 / 1.0; 

        public static final double kP = 6.5; // 6.5;
        public static final double kI = 0; // was 0.0441
        public static final double kD = 0;
        public static final double kS = 5; // 0.3
        public static final double kV = 0.1;  //try this 0;
        public static final double kA = 0.015; 
        
        public static final double acceleration = 90;

    }

    public static final class HoodConstants {

        public static final Angle MIN_HOOD_ANGLE = Degrees.of(33.0); // its really 29.359724
        public static final Angle MAX_HOOD_ANGLE = Degrees.of(71.359724);  

        public static final double sensorToMechanismRatio = 136.0 / 1; 

        public static final double kP = 13;
        public static final double kI = 0.051; 
        public static final double kD = 0.6;
        public static final double kS = 2.1; 
        public static final double kV = 0; 
        public static final double kA = 0;

        public static final double velocity = 170;
        public static final double acceleration = 170;

        public static final double stowPosition = MAX_HOOD_ANGLE.in(Degrees);
    }

    public static final class ClimbConstants {

        public static final double downPosition = 0;
        public static final double upPosition = 0;  

        public static final double kP = 0;
        public static final double kI = 0; 
        public static final double kD = 0;
        public static final double kS = 0; 
        public static final double kV = 0; 
        public static final double kA = 0; 
        public static final double kG = 0;
        
        public static final double velocity = 0;
        public static final double acceleration = 0;
    }


    public static final class Quest {

        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(12.5); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(-9);
        public static final double frontZ = Units.inchesToMeters(10); // 7.02

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(0); // negative pitch is up according to 25 code
        public static final double frontYaw = Math.toRadians(0);

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
        // Trust down to 2cm in X directio
        // Trust down to 2cm in Y direction 0.035
        // Trust down to 2 degrees rotational);
    }   

    public static final class VisionConstants {

        //FLO = Front_Left_Outside camera
        public static final double FLO_frontX = Units.inchesToMeters(9.069); // 7.495 7.176364 -7.176364
        public static final double FLO_frontY = Units.inchesToMeters(11.097);
        public static final double FLO_frontZ = Units.inchesToMeters(7.826); // 7.02

        public static final double FLO_frontRoll = Math.toRadians(0);
        public static final double FLO_frontPitch = Math.toRadians(-15.0); // -75.0 negative pitch is up according to 25 code
        public static final double FLO_frontYaw = Math.toRadians(132.273);

        //FLI = Front_Left_Outside camera
        public static final double FLI_frontX = Units.inchesToMeters(10.836); // 7.495 7.176364 -7.176364
        public static final double FLI_frontY = Units.inchesToMeters(9.396);
        public static final double FLI_frontZ = Units.inchesToMeters(7.979); // 7.02

        public static final double FLI_frontRoll = Math.toRadians(0);
        public static final double FLI_frontPitch = Math.toRadians(-30.0); // -60 negative pitch is up according to 25 code
        public static final double FLI_frontYaw = Math.toRadians(30);

        //FR = Front_Right camera
        public static final double FR_frontX = Units.inchesToMeters(10.229); // 7.495 7.176364 -7.176364
        public static final double FR_frontY = Units.inchesToMeters(-10.140);
        public static final double FR_frontZ = Units.inchesToMeters(7.825761); // 7.02

        public static final double FR_frontRoll = Math.toRadians(0);
        public static final double FR_frontPitch = Math.toRadians(-15.0); // -75.0 negative pitch is up according to 25 code
        public static final double FR_frontYaw = Math.toRadians(-131.987);

        //blueHub translations
        public static final Pose3d blueHub = new Pose3d(4.625, 4.035, 1.4304264, new Rotation3d());
        public static final Translation2d blueHubTranslation2d = blueHub.getTranslation().toTranslation2d();
        public static final Translation3d blueHubTranslation3d = blueHub.getTranslation();

        //blueDepot pose
        public static final Pose3d blueDepot = new Pose3d (0.7, 6.0, 1, new Rotation3d());

        //blueLeftBumpCorner pose
        public static final Pose3d blueLeftBumpCorner = new Pose3d (1.0, 7.74, 1, new Rotation3d()); //x4.03 and y was +1

        //blueLeftBumpCorner pose
        public static final Pose3d blueRightBumpCorner = new Pose3d (1.0, 0.0, 1, new Rotation3d()); //x4.03 and y was -1

        //redAimThreshold pose
        public static final Pose3d blueAimThreshold = new Pose3d (0.7, 2.080, 1, new Rotation3d());

        //redHub translations
        public static final Pose3d redHub = new Pose3d(11.920, 4.035, 1.4304264, new Rotation3d());
        public static final Translation2d redHubTranslation2d = redHub.getTranslation().toTranslation2d();
        public static final Translation3d redHubTranslation3d = redHub.getTranslation();

        //redAimThreshold pose
        public static final Pose3d redAimThreshold = new Pose3d (15.82, 6.0, 1, new Rotation3d());

        //redRightBumpCorner pose
        public static final Pose3d redRightBumpCorner = new Pose3d (15.0, 7.74, 1, new Rotation3d()); //12.505 and y  was + 1

        //redRightBumpCorner pose
        public static final Pose3d redLeftBumpCorner = new Pose3d (15.0, 0.552, 1, new Rotation3d()); //12.505 and y was -1

        //blueDepot pose
        public static final Pose3d redDepot = new Pose3d (15.82, 2.080, 1, new Rotation3d());

        // threshold for how close we are to the blue bump/trench for auto rotation
        public static final double blueLeftBumpOrTrenchThreshold = 2.5;
        public static final double blueRightBumpOrTrenchThreshold = 7.25;

        // threshold for how close we are to the bump/trench for auto rotation
        public static final double redLeftBumpOrTrenchThreshold = 10.0;
        public static final double redRightBumpOrTrenchThreshold = 14.0;

        public static final double topBumpTrenchEdge = 6.627;
        public static final double bottomBumpTrenchEdge = 1.43;

        //Max acceptable roll and pitch to recieve photon data
        public static final double MAX_ACCEPTABLE_PITCH = 5;
        public static final double MAX_ACCEPTABLE_ROLL = 5;
    }

    public static final class Sim {

        public static enum Mode {Real, Sim, Replay};
        public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.Real : Mode.Sim;

        public static final double fullWidth = Units.inchesToMeters(27);
        public static final double fullLength = Units.inchesToMeters(27);
        public static final double fullHeight = Units.inchesToMeters(22);
    }

    public static class FieldConstants {

        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

        // public static final Translation3d HUB_BLUE =
        //         new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        // public static final Translation3d HUB_RED =
        //         new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);
    }
}
