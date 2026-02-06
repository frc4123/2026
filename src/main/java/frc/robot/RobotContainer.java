// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.lang.Math;
// import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Oculus;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.subsystems.turret.TurretVisSim;
import frc.robot.utils.FuelSim;
import frc.robot.commands.autos.mtest;
import frc.robot.commands.autos.twoCycle;
import frc.robot.commands.autos.twoCycleDepot;
import frc.robot.commands.swerve.DriveToClimb;
import frc.robot.commands.turret.Aim;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position);

    private final SwerveRequest.FieldCentric robotStrafe = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);   
    // Field-centric strafing request using controller's d-pad

    //private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final CommandXboxController joystick = new CommandXboxController(Constants.InputConstants.kDriverControllerPort0);
    //private final CommandGenericHID m_buttonBoard = new CommandGenericHID(Constants.InputConstants.kDriverControllerPort1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Oculus oculus = new Oculus(drivetrain);
    private final Vision vision = new Vision(drivetrain, oculus);
    private final Turret turret = new Turret(drivetrain, vision);
    private final TurretVisSim turretVisSim = new TurretVisSim( () -> new Pose3d(drivetrain.getState().Pose), () -> drivetrain.getState().Speeds, vision, turret);

    private final Aim aim = new Aim(turret, drivetrain, vision);
    private final DriveToClimb leftDriveToClimb = new DriveToClimb(drivetrain, 0);
    private final DriveToClimb rightDriveToClimb = new DriveToClimb(drivetrain, 1);

    public double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();

    public RobotContainer() {
        configureBindings();

        faceAngle.HeadingController.setP(5);  
        faceAngle.HeadingController.setI(0);
        faceAngle.HeadingController.setD(0); 
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        initializeAutoChooser();
        configureFuelSim();

        turret.setDefaultCommand(aim);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.x().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/10) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed/10) 
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate/10)
            )
        );

        joystick.leftTrigger().whileTrue(leftDriveToClimb);
        joystick.rightTrigger().whileTrue(rightDriveToClimb);

        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> faceAngle
                .withVelocityX(-joystick.getLeftY() * MaxSpeed) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
                .withTargetDirection(vision.angleToFace(drivetrain.getState().Pose))
                .withTargetRateFeedforward(
                    vision.targetFF(drivetrain.getState().Pose, 
                    vision.getHub(),
                    drivetrain.getState().Speeds))
            )
        );
        //face desired angle of robot towards the Hub when B is held

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.leftTrigger().whileTrue(Commands.runOnce(SignalLogger::start));
        // joystick.rightTrigger().whileTrue(Commands.runOnce(SignalLogger::stop));
        // joystick.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on button Y press.
        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(0.1 * MaxSpeed)
            .withVelocityX(0)));

        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityY(-0.1 * MaxSpeed)
            .withVelocityX(0)));
        
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(0.1 * MaxSpeed)
            .withVelocityY(0)));

        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> robotStrafe
            .withVelocityX(-0.1 * MaxSpeed)
            .withVelocityY(0)));

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureFuelSim() {
        FuelSim instance = FuelSim.getInstance();
        instance.spawnStartingFuel();
        instance.registerRobot(
                Constants.Sim.fullWidth,
                Constants.Sim.fullLength,
                Constants.Sim.fullHeight,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds);
        instance.registerIntake(
                -Constants.Sim.fullLength,
                Constants.Sim.fullLength / 2,
                ((-Constants.Sim.fullWidth + 0.5) / 2)+ Units.inchesToMeters(7),
                (-Constants.Sim.fullWidth + 0.5) / 2 ,
                () -> turretVisSim.canIntake(),
                () -> turretVisSim.intakeFuel());
        instance.registerIntake(
                -Constants.Sim.fullLength / 2,
                Constants.Sim.fullLength / 2,
                Constants.Sim.fullWidth / 2,
                (Constants.Sim.fullWidth / 2)+ Units.inchesToMeters(7),
                () -> turretVisSim.canIntake(),
                () -> turretVisSim.intakeFuel());

        instance.start();

        if (RobotBase.isSimulation()) {
            turret.setDefaultCommand(turretVisSim.repeatedlyLaunchFuel(
                () -> {
                    ShotData shot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                        drivetrain.getState().Pose,
                        new ChassisSpeeds(),
                        turretVisSim.getTurretTarget(),
                        3
                    );
                    return shot.getExitVelocity();
                },
                () -> {
                    ShotData shot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                        drivetrain.getState().Pose,
                        new ChassisSpeeds(),
                        turretVisSim.getTurretTarget(),
                        3
                    );
                    return shot.getHoodAngle();
                },
                turret
            ));
        }
        
        SmartDashboard.putData(Commands.runOnce(() -> {
                    FuelSim.getInstance().clearFuel();
                    FuelSim.getInstance().spawnStartingFuel();
                })
                .withName("Reset Fuel")
                .ignoringDisable(true));
    }

    public void initializeAutoChooser() {
        autoChooser.setDefaultOption("5 Meter Test", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new mtest().metertest()))
        );

        autoChooser.addOption("2 Cycle Climb Right", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new twoCycle().twoCycleClimb())
        ));

        autoChooser.addOption("2 Cycle Depot Climb Left", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new twoCycleDepot().twoCycleDepotLeft())
        ));

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
