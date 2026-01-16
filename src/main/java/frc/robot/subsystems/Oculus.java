package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Oculus extends SubsystemBase{

    private final CommandSwerveDrivetrain drivetrain;

    QuestNav quest = new QuestNav();

    public Oculus(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Pose3d getRobotPose() {
        // Get the latest pose data frames from the Quest
        PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(Constants.Quest.ROBOT_TO_QUEST.inverse());
            return robotPose;
        }

        return null;
    }

    public Pose3d getQuestPose() {
         // Get the latest pose data frames from the Quest
        PoseFrame[] poseFrames = quest.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

            return questPose;
        }

        return null;
    }

    public void setRobotPose(){
        // Assume this is the requested reset pose
        Pose3d robotPose = new Pose3d( /* Some pose data */ );

        // Transform by the offset to get the Quest pose
        Pose3d questPose = robotPose.transformBy(Constants.Quest.ROBOT_TO_QUEST);

        // Send the reset operation
        quest.setPose(questPose);
    }

     @Override
    public void periodic() {
        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = quest.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(Constants.Quest.ROBOT_TO_QUEST.inverse());

                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator
                drivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, Constants.Quest.QUESTNAV_STD_DEVS);
            }
        }
    }
}
