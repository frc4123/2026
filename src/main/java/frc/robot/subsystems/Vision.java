// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Vision extends SubsystemBase{

//     private final PhotonCamera rightCamera;
//     private final Transform3d robotToRightCam;
//     private final AprilTagFieldLayout aprilTagFieldLayout;

//     // Standard deviations (tune these based on camera characteristics)
//     private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(3, 3, 3);
//     private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

//     private final StructPublisher<Pose3d> leftCamPosePublisher;
//     private final StructPublisher<Transform3d> leftCamTargetTransformPublisher;

//     private final CommandSwerveDrivetrain drivetrain;
//     private Oculus oculus;

//     public Vision(CommandSwerveDrivetrain drivetrain) {
//         this.drivetrain = drivetrain;
//         //this.aprilTagFieldLayout = loadAprilTagFieldLayout("/fields/Reefscape2025.json");

//         this.rightCamera = rightCamera;
        
//     }

// }
