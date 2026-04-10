package frc.robot.subsystems;

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
import frc.robot.Constants.Quest;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.OptionalInt;

public class Oculus extends SubsystemBase {

    private final CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
    private final Transform3d robotToQuest;
    private int loopLimiter = 0;
    private PoseFrame[] unreadFrames;
    private static Boolean trustQuest;
    StructPublisher<Pose2d> posePub;

    private double lastVisionUpdateTime = 0;

    QuestNav quest = new QuestNav();

    public Oculus() {
        trustQuest = true;

        this.robotToQuest =
                new Transform3d(
                        new Translation3d(Quest.X, Quest.Y, Quest.Z),
                        new Rotation3d(Quest.ROLL, Quest.PITCH, Quest.YAW));

        final NetworkTable questTable =
                NetworkTableInstance.getDefault().getTable("State").getSubTable("QuestNav");
        this.posePub = questTable.getStructTopic("Pose", Pose2d.struct).publish();
    }

    public Pose3d getRobotPose() {
        if (this.unreadFrames.length > 0) {
            // Get the most recent Quest pose
            final Pose3d questPose = this.unreadFrames[this.unreadFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            final Pose3d robotPose = questPose.transformBy(this.robotToQuest.inverse());
            return robotPose;
        }

        return null;
    }

    public Pose3d getQuestPose() {
        if (this.unreadFrames.length > 0) {
            // Get the most recent Quest pose
            final Pose3d questPose = this.unreadFrames[this.unreadFrames.length - 1].questPose3d();
            return questPose;
        }
        return null;
    }

    public void setRobotPose() {
        // Transform by the offset to get the Quest pose
        final double now = Timer.getFPGATimestamp();
        if (now - this.lastVisionUpdateTime < 20) return; // 20s refresh cap

        this.lastVisionUpdateTime = now;

        final Pose2d pose2d = this.swerve.getState().Pose;
        final Pose3d questPose = new Pose3d(pose2d).transformBy(this.robotToQuest);

        this.quest.setPose(questPose);
    }

    public boolean isQuestNavConnected() {
        // You might need to check NetworkTables or add a timeout mechanism
        final PoseFrame[] frames = this.unreadFrames;
        return frames != null && frames.length > 0;
    }

    public void updateSwerve() {
        // if there are no questFrames then dont crash the robot code

        if (this.unreadFrames == null || this.unreadFrames.length <= 0) {
            return;
        }
        // Get the latest pose data frames from the Quest
        // Loop over the pose data frames and send them to the pose estimator

        final PoseFrame latestFrame = this.unreadFrames[this.unreadFrames.length - 1];

        if (latestFrame.isTracking()) {
            final Pose3d questPose = latestFrame.questPose3d();
            final double timestamp = latestFrame.dataTimestamp();

            final Pose3d robotPose = questPose.transformBy(this.robotToQuest.inverse());

            // Compare Quest pose against current swerve odometry estimate
            final double deviation =
                    this.swerve
                            .getState()
                            .Pose
                            .getTranslation()
                            .getDistance(robotPose.toPose2d().getTranslation());

            // Hard reject if Quest disagrees with odometry too much
            if (deviation > 0.5) return;

            this.swerve.addVisionMeasurement(
                    robotPose.toPose2d(), timestamp, Quest.QUESTNAV_STD_DEVS);
        }

        // for (PoseFrame questFrame : unreadFrames) {
        // // Make sure the Quest was tracking the pose for this frame
        // if (questFrame.isTracking()) {
        // // Get the pose of the Quest
        // Pose3d questPose = questFrame.questPose3d();
        // // Get timestamp for when the data was sent
        // double timestamp = questFrame.dataTimestamp();

        // // Transform by the mount pose to get your robot pose
        // Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

        // // You can put some sort of filtering here if you would like!

        // // Add the measurement to our estimator
        // swerve.addVisionMeasurement(
        // robotPose.toPose2d(),
        // timestamp,
        // Quest.QUESTNAV_STD_DEVS
        // );
        // }
        // }
    }

    public static Boolean trustQuest() {
        return trustQuest;
    }

    public void publishQuestStatus() {
        final OptionalInt questBattery = this.quest.getBatteryPercent();
        // boolean questTrackingStatus = quest.isTracking();
        int questBatteryInt;

        if (this.quest.isConnected() && this.loopLimiter % 10 == 0) {
            questBatteryInt = questBattery.getAsInt();
            SmartDashboard.putString("Oculus Quest Battery", questBatteryInt + "%");
            // SmartDashboard.putBoolean("Is Quest Tracking", questTrackingStatus);
        } else if (!this.quest.isConnected()) {
            SmartDashboard.putString("Oculus Quest Battery", "unable to be retrieved");
        }
    }

    // public void publishQuestState(){
    // Pose3d questPose = getQuestPose();
    // if(questPose != null) {
    // Pose2d questPose2d = questPose.toPose2d();
    // posePub.set(questPose2d);
    // }
    // }

    @Override
    public void periodic() {

        this.publishQuestStatus();
        trustQuest = SmartDashboard.getBoolean("Trust Quest", true);
        this.unreadFrames = this.quest.getAllUnreadPoseFrames();
        if (this.isQuestNavConnected()) {
            this.setRobotPose();
            if (trustQuest) {
                this.updateSwerve();
            }
        }

        this.loopLimiter++;
    }
}
