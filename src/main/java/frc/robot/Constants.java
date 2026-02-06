package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static final class CanIdCanivore { 

        public static final String ID = "Good Boy CANivore 10";

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
        public static final int Turret_Encoder2 = 18;

    }

    public static class IntakeConstants {
        public static final int LEFT_RACK_ID = 0;
        public static final int RIGHT_RACK_ID = 0;
        public static final int LEFT_SPIN_ID = 0;
        public static final int RIGHT_SPIN_ID = 0;


        public static final Distance STOW_POS = Inches.of(0);
        public static final Distance DEPLOY_POS = Inches.of(10.875);
        public static final Voltage SPIN_VOLTAGE = Volts.of(3);

        public static final LinearVelocity MIN_SWITCH_ROBOT_VELOCITY = MetersPerSecond.of(0.5);

        public static final double VEL_MULTIPLIER = 70.0; // multiplies goal velocity for targetting
        public static final double VEL_POWER = 0.3; // raises goal velocity to power
        public static final LinearVelocity BASE_VEL = InchesPerSecond.of(50); // added to final velocity
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

    public static final class TurretConstants {
        public static final double stowPosition = 0;
        public static final double velocity = 1; // when your confident that the pid always reaches setpoint then jack ts up
        public static final double acceleration = 2;

        public static final double kP = 300;
        public static final double kI = 6.7;
        public static final double kD = 0;
        public static final double kS = 0; // was 0.239 but dont need bc already moving so 0 for now
        public static final double kV = 4.4; // 0.64 / 0.15 according to calculation but was innacurate by a little bit irl
        public static final double kA = 0;

        public static final int mechanismMinRange = -1; // -360 degrees
        public static final int mechanismMaxRange = 1; // +360 degrees
        // this makes total of 720 degrees rotation^^^^

        public static final double motorToTurretRatio = (48.0/9.0) * (180.0/24.0); 

        public static final double rotorToEncoder1Ratio = 48.0 / 9.0;
        public static final double sensorToMechanismRatio = 180.0 / 24.0;

        
        public static final double turretGearTeeth = 180.0;     // Turret gear (drives both encoders)
        public static final double encoder1Teeth = 24.0;      // Gear on Hex Shaft A that connects to turret
        public static final double encoder2Teeth = 50.0;      // Gear on Hex Shaft B that connects to turret

        public static final double encoder1Offset = -0.575928;
        public static final double encoder2Offset = -0.597412;
        //TODO: if the wrap happens to be near the zero measurement (within hundredths check yams for interval confirmation), then RESEAT CANCODERS
                
        public static final double coverageMargin = 1.2;
        public static final int minTeeth = 1;
        public static final int maxTeeth = 1;
        public static final int maxIterations = 30;

        //TODO: change all tssssss

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

    public static final class Quest {

        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(11.5); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(-4.5);
        public static final double frontZ = Units.inchesToMeters(6.75); // 7.02

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(0); // negative pitch is up according to 25 code
        public static final double frontYaw = Math.toRadians(0);

        public static final Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.035);
        // Trust down to 2cm in X directio
        // Trust down to 2cm in Y direction 0.035
        // Trust down to 2 degrees rotational);
    }   

    public static final class VisionConstants {

        //Front Forward Camera Translation and Angle
        public static final double frontX = Units.inchesToMeters(-11.5); // 7.495 7.176364 -7.176364
        public static final double frontY = Units.inchesToMeters(0);
        public static final double frontZ = Units.inchesToMeters(8.186116); // 7.02

        public static final Pose3d blueHub = new Pose3d(4.625, 4.035, 1.4304264, new Rotation3d());
        public static final Translation2d blueHubTranslation2d = blueHub.getTranslation().toTranslation2d();
        public static final Translation3d blueHubTranslation3d = blueHub.getTranslation();

        public static final Pose3d redHub = new Pose3d(11.920, 4.035, 1.4304264, new Rotation3d());
        public static final Translation2d redHubTranslation2d = redHub.getTranslation().toTranslation2d();
        public static final Translation3d redHubTranslation3d = redHub.getTranslation();

        public static final double MAX_ACCEPTABLE_PITCH = 6;
        public static final double MAX_ACCEPTABLE_ROLL = 6;


        // Camera transforms
        public static final Transform3d robotToCam = new Transform3d(
            new Translation3d(
                Constants.VisionConstants.frontX,
                Constants.VisionConstants.frontY,
                Constants.VisionConstants.frontZ),
            new Rotation3d(
                Constants.VisionConstants.frontRoll,
                Constants.VisionConstants.frontPitch,
                Constants.VisionConstants.frontYaw)
        );

        public static final double frontRoll = Math.toRadians(0);
        public static final double frontPitch = Math.toRadians(-60); // negative pitch is up according to 25 code
        public static final double frontYaw = Math.toRadians(180);
    }

    public static final class Sim{

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
