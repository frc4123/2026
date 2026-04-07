package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;

public class Telemetry {

    private final Field2d m_field = new Field2d();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("DriveState");

    private final StructPublisher<Pose2d> drivePose =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    public Telemetry(double maxSpeed) {
        SmartDashboard.putData("Robot Field", m_field);
    }

    public void telemeterize(SwerveDriveState state) {
        drivePose.set(CommandSwerveDrivetrain.getInstance().getState().Pose);
        m_field.setRobotPose(CommandSwerveDrivetrain.getInstance().getState().Pose);
        Logger.recordOutput("Drive/Pose", CommandSwerveDrivetrain.getInstance().getState().Pose);
    }
}