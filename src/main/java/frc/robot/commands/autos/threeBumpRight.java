package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class threeBumpRight extends SubsystemBase {
    public Command threeBumpAuto() {
        return AutoBuilder.buildAuto("3BumpRight");
    }
}
