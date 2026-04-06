package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalInt;

public class LocalizerVisionIOQuestNav implements LocalizerVisionIO {
    private static LocalizerVisionIOQuestNav instance;

    public static LocalizerVisionIOQuestNav getInstance(final Transform3d robotToCamera) {
        if (instance == null) {
            instance = new LocalizerVisionIOQuestNav(robotToCamera);
        }
        return instance;
    }

    private final QuestNav questNav = new QuestNav();
    private final Transform3d robotToCamera;
    private int statusPublishingLimiter = 0;

    private LocalizerVisionIOQuestNav(final Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        this.registerCallbacks();
    }

    private void registerCallbacks() {
        this.questNav.onConnected(() -> System.out.println("Quest connected!"));
        this.questNav.onDisconnected(
                () -> DriverStation.reportWarning("Quest disconnected!", false));
        this.questNav.onTrackingAcquired(() -> System.out.println("Quest tracking acquired!"));
        this.questNav.onTrackingLost(
                () -> DriverStation.reportWarning("Quest tracking lost!", false));
        this.questNav.onLowBattery(
                20,
                level -> DriverStation.reportWarning("Quest battery low: " + level + "%", false));
    }

    private void publishQuestStatus() {
        this.statusPublishingLimiter = (this.statusPublishingLimiter + 1) % 50;
        if (this.statusPublishingLimiter != 0) {
            return;
        }
        // Not putting this inside of the isConnected block in #updateInputs since I want to have
        // the counter execute every tick
        if (!this.questNav.isConnected()) {
            SmartDashboard.putString("Oculus Quest Battery", "not connected");
            return;
        }
        final OptionalInt battery = this.questNav.getBatteryPercent();

        SmartDashboard.putString(
                "Oculus Quest Battery",
                battery.isPresent() ? battery.getAsInt() + "%" : "unable to be retrieved");
    }

    @Override
    public void setRobotPose(final Pose3d pose) {
        this.questNav.setPose(pose.transformBy(this.robotToCamera));
    }

    @Override
    public void updateInputs(final LocalizerIOInputs inputs) {
        this.questNav.commandPeriodic();
        this.publishQuestStatus();
        final boolean shouldTrustQuest = SmartDashboard.getBoolean("Trust Quest", false);

        // If we aren't trusting the quest, treat it as not connected
        inputs.connected = shouldTrustQuest && this.questNav.isConnected();

        final List<LocalizerPoseObservation> poseObservations = new ArrayList<>();
        final PoseFrame[] poseFrames = this.questNav.getAllUnreadPoseFrames();

        if (poseFrames != null && poseFrames.length > 0) {
            final PoseFrame latest = poseFrames[poseFrames.length - 1];
            if (latest.isTracking()) {
                poseObservations.add(
                        new LocalizerPoseObservation(
                                latest.dataTimestamp(),
                                latest.questPose3d().transformBy(this.robotToCamera.inverse())));
            }
        }

        inputs.poseObservations = poseObservations.toArray(new LocalizerPoseObservation[0]);
    }
}
