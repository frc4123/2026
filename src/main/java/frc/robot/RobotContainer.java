// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import java.lang.Math;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Oculus;
import frc.robot.subsystems.SevenEleven;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.subsystems.turret.TurretVisSim;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShiftHelpers;
import frc.robot.utils.Target;
import frc.robot.Constants.InputConstants;
import frc.robot.Constants.Sim;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.Sim.Mode;
import frc.robot.commands.autos.CityBoyLeft;
import frc.robot.commands.autos.CityBoyRight;
import frc.robot.commands.autos.MadTown;
import frc.robot.commands.autos.mtest;
import frc.robot.commands.autos.orbit;
import frc.robot.commands.hood.AvoidDecapitation;
import frc.robot.commands.hood.HoodAim;
import frc.robot.commands.intakeArm.ForceIntakeArmMid;
import frc.robot.commands.intakeArm.IntakeArmIn;
import frc.robot.commands.intakeArm.IntakeArmInMid;
import frc.robot.commands.intakeArm.IntakeArmInSlow;
import frc.robot.commands.intakeArm.IntakeArmMid;
import frc.robot.commands.intakeArm.IntakeArmOut;
import frc.robot.commands.intakeRoller.AutoIntakeRollerIn;
import frc.robot.commands.intakeRoller.IntakeReverse;
//import frc.robot.commands.intakeArm.IntakeShimmy;
import frc.robot.commands.intakeRoller.IntakeRollerIn;
import frc.robot.commands.intakeRoller.IntakeRollerShimmy;
import frc.robot.commands.intakeRoller.IntakeRollerStop;
import frc.robot.commands.sevenEleven.RollReverse;
import frc.robot.commands.sevenEleven.RollStop;
// import frc.robot.commands.sevenEleven.RollHigh;
// import frc.robot.commands.sevenEleven.RollLow;
// import frc.robot.commands.sevenEleven.RollMid;
import frc.robot.commands.shooter.SetShooterVelocity;
// import frc.robot.commands.swerve.DriveToClimb;
import frc.robot.commands.turret.Aim;
import frc.robot.commands.uptake.UptakeStop;
import frc.robot.commands.uptake.UptakeUp;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    final Rotation2d[] snappedAngle = {new Rotation2d()};

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
             // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.05);

    private final SwerveRequest.FieldCentric robotStrafe = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);   
    // Field-centric strafing request using controller's d-pad

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final CommandXboxController joystick = new CommandXboxController(InputConstants.kDriverControllerPort0);
    private final CommandGenericHID m_buttonBoard = new CommandGenericHID(InputConstants.kDriverControllerPort1);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Oculus oculus = new Oculus();
    private final Vision vision = new Vision(oculus);
    private final Turret turret = new Turret(drivetrain);
    private final TurretVisSim turretVisSim = new TurretVisSim( () -> new Pose3d(drivetrain.getState().Pose), () -> drivetrain.getState().Speeds, vision, turret);
    private final IntakeRoller intakeRollers = new IntakeRoller();
    private final SevenEleven sevenEleven = new SevenEleven();
    private final IntakeArm intakeArm = new IntakeArm();
    private final Hood hood = new Hood();
    private final Shooter shooter = new Shooter();
    private final Uptake uptake = new Uptake();
    // private final Climb climb = new Climb();

    private final Aim aim = new Aim(turret, drivetrain);
    // private final DriveToClimb leftDriveToClimb = new DriveToClimb(drivetrain, 0);
    // private final DriveToClimb rightDriveToClimb = new DriveToClimb(drivetrain, 1);
    private final IntakeRollerIn intakeRollersIn = new IntakeRollerIn(intakeRollers, intakeArm);
    private final AutoIntakeRollerIn autoIntakeRollerIn = new AutoIntakeRollerIn(intakeRollers, intakeArm);
    private final IntakeRollerStop intakeRollersStop = new IntakeRollerStop(intakeRollers);
    private final IntakeReverse intakeReverse = new IntakeReverse(intakeRollers, intakeArm);
    // private final IntakeRollerShimmy intakeRollerShimmy = new IntakeRollerShimmy(intakeRollers, intakeArm);
    //private final Roll roll = new Roll(sevenEleven);
    // private final RollLow rollLow = new RollLow(sevenEleven);
    // private final RollMid rollMid = new RollMid(sevenEleven);
    // private final RollHigh rollHigh = new RollHigh(sevenEleven);
    // private final RepeatCommand rollerPulse =
    //     new RepeatCommand(
    //         rollLow.withTimeout(0.5)
    //         .andThen(rollMid.withTimeout(0.5))
    //         .andThen(rollHigh.withTimeout(1)
    //     )
    // );
    private final IntakeArmIn intakeArmIn = new IntakeArmIn(intakeArm, intakeRollers);
    private final IntakeArmInSlow intakeArmInSlow = new IntakeArmInSlow(intakeArm, intakeRollers);
    private final IntakeArmInMid intakeArmInMid = new IntakeArmInMid(intakeArm, intakeRollers);
    private final IntakeArmOut intakeArmOut = new IntakeArmOut(intakeArm);
    //private final IntakeArmMid intakeArmMid = new IntakeArmMid(intakeArm, intakeRollers);
    private final ForceIntakeArmMid forceIntakeArmMid = new ForceIntakeArmMid(intakeArm);
    //private final IntakeShimmy intakeShimmy = new IntakeShimmy(intakeArm, intakeRollers);
    private final RollReverse rollReverse = new RollReverse(sevenEleven);
    private final RollStop rollStop = new RollStop(sevenEleven);
    private final HoodAim hoodAim = new HoodAim(hood);
    private final AvoidDecapitation avoidDecapitation = new AvoidDecapitation(hood);
    private final SetShooterVelocity setShooterVelocity = new SetShooterVelocity(shooter);
    private final UptakeUp uptakeUp = new UptakeUp(uptake, turret, sevenEleven, shooter);
    private final UptakeStop uptakeStop = new UptakeStop(uptake, shooter);
    //private final UptakeReverse uptakeReverse = new UptakeReverse(uptake);
    // private final ClimbUp climbUp = new ClimbUp(climb);
    // private final ClimbDown climbDown = new ClimbDown(climb);
    // private final ClimbTest climbTest = new ClimbTest(climb);

    public double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();

    public RobotContainer() {
        configureBindings();

        drivetrain.setOnPoseResetCallback(pose -> {
            if (oculus.isQuestNavConnected()) {
                oculus.setRobotPose();
            }
        });

        faceAngle.HeadingController.setP(3.4123);// 3.1, 3.4123
        faceAngle.HeadingController.setI(0);
        faceAngle.HeadingController.setD(0); 
        faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        if(Sim.CURRENT_MODE == Mode.Sim){
            configureFuelSim();
        }

        turret.setDefaultCommand(aim);
        hood.setDefaultCommand(hoodAim);
        shooter.setDefaultCommand(setShooterVelocity);
        //sevenEleven.setDefaultCommand(rollerPulse);

        NamedCommands.registerCommand("ArmIn", intakeArmIn);
        NamedCommands.registerCommand("ArmSlowIn", intakeArmInSlow);
        NamedCommands.registerCommand("ArmOut", intakeArmOut);
        NamedCommands.registerCommand("IntakeShimmy", new WaitCommand(1.8).andThen(
            new RepeatCommand(
                new IntakeArmMid(intakeArm, intakeRollers)
                .andThen(new WaitCommand(0.7))
                .andThen(new ParallelCommandGroup(new IntakeArmOut(intakeArm), new IntakeRollerShimmy(intakeRollers, intakeArm))
                .andThen(new WaitCommand(0.7)))
            )
        ));
        NamedCommands.registerCommand("Aim", aim);
        NamedCommands.registerCommand("IntakeIn", autoIntakeRollerIn);
        NamedCommands.registerCommand("IntakeReverse", intakeReverse);
        NamedCommands.registerCommand("IntakeStop", intakeRollersStop);
        NamedCommands.registerCommand("Uptake", uptakeUp);
        NamedCommands.registerCommand("HoodDown", avoidDecapitation);
        NamedCommands.registerCommand("UptakeStop", uptakeStop);
        // NamedCommands.registerCommand("ClimbUp", climbUp);
        // NamedCommands.registerCommand("ClimbDown", climbDown);

        initializeAutoChooser();
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        return value;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-applyDeadband(joystick.getLeftY(), SwerveConstants.driverDeadband) * MaxSpeed)
                    .withVelocityY(-applyDeadband(joystick.getLeftX(), SwerveConstants.driverDeadband) * MaxSpeed)
                    .withRotationalRate(-applyDeadband(joystick.getRightX(), SwerveConstants.driverDeadband) * MaxAngularRate)
            )
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        joystick.x().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed/10) 
                .withVelocityY(-joystick.getLeftX() * MaxSpeed/10) 
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate/10)
            )
        );

        // joystick.leftBumper().whileTrue(leftDriveToClimb);
        // joystick.rightBumper().whileTrue(rightDriveToClimb);

        // Snapshot angle on press, store it
        

        joystick.b().onTrue(
            Commands.runOnce(() -> {
                snappedAngle[0] = Target.getTrenchAngle(
                    drivetrain.getState().Pose.getTranslation().getX()
                );
            })
        );

        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> faceAngle
                .withVelocityX(-joystick.getLeftY() * MaxSpeed / 2)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed / 2)
                .withTargetDirection(snappedAngle[0])
            )
        );

        // joystick.y().onTrue(
        //     Commands.runOnce(() -> {
        //         snappedAngle[0] = Target.getBumpAngle(
        //             drivetrain.getState().Pose.getTranslation().getX()
        //         );
        //     })
        // );

        // joystick.y().whileTrue(
        //     drivetrain.applyRequest(() -> faceAngle
        //         .withVelocityX(-joystick.getLeftY() * MaxSpeed / 2)
        //         .withVelocityY(-joystick.getLeftX() * MaxSpeed / 2)
        //         .withTargetDirection(snappedAngle[0])
        //     )
        // );

        joystick.y().onTrue(drivetrain.applyRequest(() -> brake));

        //face desired angle of robot towards the Hub when B is held

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on button Y press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> {
                double leftY = -joystick.getLeftY();
                double leftX = -joystick.getLeftX();
                
                double magnitude = Math.sqrt(leftX * leftX + leftY * leftY);
                Rotation2d targetDirection;
                
                if (magnitude > 0.05) { // Only update rotation when stick is moved
                    targetDirection = new Rotation2d(joystick.getLeftY(), joystick.getLeftX());

                    return faceAngle
                        .withVelocityX(leftY * MaxSpeed * 0.54123) // was 0.6
                        .withVelocityY(leftX * MaxSpeed * 0.54123) //was 0.6
                        .withTargetDirection(targetDirection);

                } else {
                    return robotStrafe
                        .withVelocityX(0) // was 0.6
                        .withVelocityY(0);
                }
                
                
            })
        );

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

        drivetrain.registerTelemetry(logger::telemeterize);

        //  --------- Subsystem COMMANDS ---------- // (non swerve subsystem)

        joystick.a().onTrue(intakeRollersIn);
        joystick.a().onFalse(intakeRollersStop);
        
        joystick.a().onTrue(intakeArmOut);
        joystick.a().onFalse(intakeRollersStop);

        joystick.b().whileTrue(avoidDecapitation);
        joystick.b().onTrue(intakeRollersIn);
        joystick.b().onFalse(intakeRollersStop);

        joystick.rightTrigger().onTrue(uptakeUp);
        joystick.rightTrigger().onFalse(uptakeStop);

        joystick.leftTrigger().onTrue(forceIntakeArmMid);
        joystick.leftTrigger().onFalse(intakeArmOut);

        //TODO make a shot that is independent of pose

        joystick.leftStick().onTrue(intakeArmIn);

        joystick.rightStick().onTrue(avoidDecapitation);
        joystick.rightStick().onFalse(hoodAim);

        // joystick.button(8).onTrue(manualReset);
        // joystick.button(8).onFalse(hoodAim);
        
        // ---------- Buttonboard Commands ------------ //        

        m_buttonBoard.button(1).onTrue(uptakeUp);
        m_buttonBoard.button(1).onFalse(uptakeStop);

        m_buttonBoard.button(2).onTrue(intakeReverse);
        m_buttonBoard.button(2).onTrue(intakeArmOut);
        m_buttonBoard.button(2).onTrue(rollReverse);
        m_buttonBoard.button(2).onFalse(rollStop);

        m_buttonBoard.button(3).onTrue(uptakeUp);
        m_buttonBoard.button(3).onFalse(intakeArmOut);
        m_buttonBoard.button(3).onFalse(uptakeStop);
        m_buttonBoard.button(3).onTrue(intakeArmInMid);

        m_buttonBoard.button(4).onTrue(uptakeUp);
        m_buttonBoard.button(4).onFalse(uptakeStop);
        m_buttonBoard.button(4).onFalse(intakeArmOut);
        m_buttonBoard.button(4).whileTrue(new WaitCommand(1.5).andThen(new IntakeArmInSlow(intakeArm, intakeRollers))
            .andThen(new WaitCommand(0.75)).andThen(new IntakeArmOut(intakeArm))
            .andThen(new WaitCommand(0.75)).andThen(new ForceIntakeArmMid(intakeArm)));
        
        Trigger upcomingShiftWarning = new Trigger(() ->
            ShiftHelpers.isFiveSecBeforeShiftChange(Timer.getMatchTime()) && !ShiftHelpers.currentShiftIsYours()
        );

        Trigger endingShiftWarning = new Trigger(() ->
            ShiftHelpers.isTwelveSecBeforeShiftChange(Timer.getMatchTime()) && ShiftHelpers.currentShiftIsYours()
        );

        upcomingShiftWarning.whileTrue(
            Commands.run(() ->
                joystick.setRumble(RumbleType.kBothRumble, 1.0)
            )
        );

        endingShiftWarning.whileTrue(
            Commands.run(() ->
                joystick.setRumble(RumbleType.kBothRumble, 1.0)
            )
        );

        upcomingShiftWarning.onFalse(
            Commands.runOnce(() ->
                joystick.setRumble(RumbleType.kBothRumble, 0)
            )
        );

        endingShiftWarning.onFalse(
            Commands.run(() ->
                joystick.setRumble(RumbleType.kBothRumble, 0)
            )
        );
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
        // instance.registerIntake(
        //     -Constants.Sim.fullLength,
        //     Constants.Sim.fullLength / 2,
        //     ((-Constants.Sim.fullWidth + 0.5) / 2) + Units.inchesToMeters(7),
        //     (-Constants.Sim.fullWidth + 0.5) / 2 ,
        //     () -> turretVisSim.canIntake(),
        //     () -> turretVisSim.intakeFuel());
        // instance.registerIntake(
        //     -Constants.Sim.fullLength / 2,
        //     Constants.Sim.fullLength / 2,
        //     Constants.Sim.fullWidth / 2,
        //     (Constants.Sim.fullWidth / 2) + Units.inchesToMeters(7),
        //     () -> turretVisSim.canIntake(),
        //     () -> turretVisSim.intakeFuel());
        instance.registerIntake(
            -Constants.Sim.fullLength / 2,
            -Constants.Sim.fullLength,
            -Constants.Sim.fullWidth / 2,
            Constants.Sim.fullWidth / 2,
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
        autoChooser.setDefaultOption("super secret auto", 
        new WaitCommand(5)
        .andThen(new ParallelRaceGroup(
            new IntakeArmOut(intakeArm),
            // new IntakeRollerIn(intakeRollers, intakeArm),
            new WaitCommand(2)))
        .andThen(uptakeUp));

        autoChooser.addOption("City Boy Left", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new CityBoyLeft().cityBoyLeft())
        ));

        autoChooser.addOption("City Boy Right", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new CityBoyRight().cityBoyRight())
        ));

        autoChooser.addOption("MadTown Left", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new MadTown().madTownLeft())
        ));

        autoChooser.addOption("Orbit Right", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new orbit().orbitRight())
        ));

        autoChooser.addOption("Orbit Right Delay", new SequentialCommandGroup(
            new WaitCommand(4).andThen(new orbit().orbitRight())
        ));

        autoChooser.addOption("5m test", new ParallelCommandGroup(
            new WaitCommand(0.01),
            new SequentialCommandGroup(new mtest().metertest())
        ));

        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
