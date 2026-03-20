package frc.robot.subsystems;

import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Oculus extends SubsystemBase{

    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final Transform3d robotToQuest;
    private int loopLimiter = 0;
    private PoseFrame[] unreadFrames;
    StructPublisher<Pose2d> posePub;

    private double lastVisionUpdateTime = 0;

    QuestNav quest = new QuestNav();

    public Oculus() {

        robotToQuest = new Transform3d(
            new Translation3d(
                Constants.Quest.frontX,
                Constants.Quest.frontY,
                Constants.Quest.frontZ),
            new Rotation3d(
                Constants.Quest.frontRoll,
                Constants.Quest.frontPitch,
                Constants.Quest.frontYaw)
        );

        NetworkTable questTable = NetworkTableInstance.getDefault()
            .getTable("State")
            .getSubTable("QuestNav");
        posePub = questTable.getStructTopic("Pose", Pose2d.struct).publish();
    }

    public Pose3d getRobotPose() {
        if (unreadFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = unreadFrames[unreadFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());
            return robotPose;
        }

        return null;
    }

    public Pose3d getQuestPose() {
        if (unreadFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = unreadFrames[unreadFrames.length - 1].questPose3d();
            return questPose;
        }
        return null;
    }

    public void setRobotPose(Pose3d pose){
        // Transform by the offset to get the Quest pose
        double now = Timer.getFPGATimestamp();
        if (now - lastVisionUpdateTime < 0.5) return; // ~50 Hz cap

        lastVisionUpdateTime = now;

        Pose3d questPose = pose.transformBy(robotToQuest);
        // Send the reset operation
        quest.setPose(questPose);
    }

    public boolean isQuestNavConnected() {
        // You might need to check NetworkTables or add a timeout mechanism
        PoseFrame[] frames = unreadFrames;
        return frames != null && frames.length > 0;
    }

    public void updateSwerve(){
        //if there are no questFrames then dont crash the robot code
        
        if( unreadFrames == null || unreadFrames.length <= 0) {return;}
        // Get the latest pose data frames from the Quest
        // Loop over the pose data frames and send them to the pose estimator

        PoseFrame latestFrame = unreadFrames[unreadFrames.length - 1];

        if (latestFrame.isTracking()) {
            Pose3d questPose = latestFrame.questPose3d();
            double timestamp = latestFrame.dataTimestamp();

            Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

            swerve.addVisionMeasurement(
                robotPose.toPose2d(),
                timestamp,
                Constants.Quest.QUESTNAV_STD_DEVS
            );
        }
        // for (PoseFrame questFrame : unreadFrames) {
        //     // Make sure the Quest was tracking the pose for this frame
        //     if (questFrame.isTracking()) {
        //         // Get the pose of the Quest
        //         Pose3d questPose = questFrame.questPose3d();
        //         // Get timestamp for when the data was sent
        //         double timestamp = questFrame.dataTimestamp();

        //         // Transform by the mount pose to get your robot pose
        //         Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

        //         // You can put some sort of filtering here if you would like!

        //         // Add the measurement to our estimator
        //         swerve.addVisionMeasurement(
        //             robotPose.toPose2d(),
        //             timestamp,
        //             Constants.Quest.QUESTNAV_STD_DEVS
        //         );
        //     }
        // }
    }
    
    public void publishQuestStatus(){
        OptionalInt questBattery = quest.getBatteryPercent();
        // boolean questTrackingStatus = quest.isTracking();
        int questBatteryInt;

        if (quest.isConnected() && loopLimiter % 10 == 0) {
            questBatteryInt = questBattery.getAsInt();
            SmartDashboard.putString("Oculus Quest Battery", questBatteryInt + "%");
            // SmartDashboard.putBoolean("Is Quest Tracking", questTrackingStatus);
        } else if (!quest.isConnected()) { 
            SmartDashboard.putString("Oculus Quest Battery", "unable to be retrieved");
        }
    }

    // public void publishQuestState(){
    //     Pose3d questPose = getQuestPose();
    //     if(questPose != null) {
    //         Pose2d questPose2d = questPose.toPose2d();
    //         posePub.set(questPose2d);
    //     }
    // }

    @Override
    public void periodic() {
        unreadFrames = quest.getAllUnreadPoseFrames();
        updateSwerve();
        publishQuestStatus();
        loopLimiter++;
        //publishQuestState();
    }
}