// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.InputConstants;
import frc.robot.Constants.Sim;
import frc.robot.Constants.Sim.Mode;
import frc.robot.Constants.SwerveConstants;
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
// import frc.robot.commands.sevenEleven.RollHigh;
// import frc.robot.commands.sevenEleven.RollLow;
// import frc.robot.commands.sevenEleven.RollMid;
import frc.robot.commands.shooter.SetShooterVelocity;
// import frc.robot.commands.swerve.DriveToClimb;
import frc.robot.commands.turret.Aim;
import frc.robot.commands.uptake.UptakeStop;
import frc.robot.commands.uptake.UptakeUp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Oculus;
import frc.robot.subsystems.SevenEleven;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculator;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.subsystems.turret.TurretVisSim;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShiftHelpers;
import frc.robot.utils.Target;

public class RobotContainer {
        private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts
                                                                                                        // desired top
                                                                                                        // speed
        private final double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a
                                                                                                            // rotation
                                                                                                            // per
                                                                                                            // second
                                                                                                            // max
                                                                                                            // angular
                                                                                                            // velocity

        final Rotation2d[] snappedAngle = { new Rotation2d() };

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(this.MaxSpeed * 0.05).withRotationalDeadband(this.MaxAngularRate * 0.05) // Add a
                                                                                                               // 10%
                                                                                                               // deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new
        // SwerveRequest.PointWheelsAt();

        private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(DriveRequestType.Velocity)
                        .withSteerRequestType(SteerRequestType.Position)
                        .withDeadband(this.MaxSpeed * 0.05)
                        .withRotationalDeadband(this.MaxAngularRate * 0.05);

        private final SwerveRequest.FieldCentric robotStrafe = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        // Field-centric strafing request using controller's d-pad

        private final Telemetry logger = new Telemetry(this.MaxSpeed);

        private final SendableChooser<Command> autoChooser = new SendableChooser<>();

        private final CommandXboxController joystick = new CommandXboxController(InputConstants.kDriverControllerPort0);
        private final CommandGenericHID m_buttonBoard = new CommandGenericHID(InputConstants.kDriverControllerPort1);

        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final Oculus oculus = new Oculus();
        // private final Vision vision = new Vision();
        private final Turret turret = new Turret(this.drivetrain);
        private final TurretVisSim turretVisSim = new TurretVisSim(() -> new Pose3d(this.drivetrain.getState().Pose),
                        this.turret);
        private final IntakeRoller intakeRollers = new IntakeRoller();
        private final SevenEleven sevenEleven = new SevenEleven();
        private final IntakeArm intakeArm = new IntakeArm();
        private final Hood hood = new Hood();
        private final Shooter shooter = new Shooter();
        private final Uptake uptake = new Uptake();
        // private final Climb climb = new Climb();

        private final Aim aim = new Aim(this.turret, this.drivetrain);
        // private final DriveToClimb leftDriveToClimb = new DriveToClimb(drivetrain,
        // 0);
        // private final DriveToClimb rightDriveToClimb = new DriveToClimb(drivetrain,
        // 1);
        private final IntakeRollerIn intakeRollersIn = new IntakeRollerIn(this.intakeRollers, this.intakeArm);
        private final AutoIntakeRollerIn autoIntakeRollerIn = new AutoIntakeRollerIn(this.intakeRollers,
                        this.intakeArm);
        private final IntakeRollerStop intakeRollersStop = new IntakeRollerStop(this.intakeRollers);
        private final IntakeReverse intakeReverse = new IntakeReverse(this.intakeRollers, this.intakeArm);
        // private final IntakeRollerShimmy intakeRollerShimmy = new
        // IntakeRollerShimmy(intakeRollers, intakeArm);
        // private final Roll roll = new Roll(sevenEleven);
        // private final RollLow rollLow = new RollLow(sevenEleven);
        // private final RollMid rollMid = new RollMid(sevenEleven);
        // private final RollHigh rollHigh = new RollHigh(sevenEleven);
        // private final RepeatCommand rollerPulse =
        // new RepeatCommand(
        // rollLow.withTimeout(0.5)
        // .andThen(rollMid.withTimeout(0.5))
        // .andThen(rollHigh.withTimeout(1)
        // )
        // );
        private final IntakeArmIn intakeArmIn = new IntakeArmIn(this.intakeArm, this.intakeRollers);
        private final IntakeArmInSlow intakeArmInSlow = new IntakeArmInSlow(this.intakeArm, this.intakeRollers);
        private final IntakeArmInMid intakeArmInMid = new IntakeArmInMid(this.intakeArm, this.intakeRollers);
        private final IntakeArmOut intakeArmOut = new IntakeArmOut(this.intakeArm);
        // private final IntakeArmMid intakeArmMid = new IntakeArmMid(intakeArm,
        // intakeRollers);
        private final ForceIntakeArmMid forceIntakeArmMid = new ForceIntakeArmMid(this.intakeArm);
        // private final IntakeShimmy intakeShimmy = new IntakeShimmy(intakeArm,
        // intakeRollers);
        private final RollReverse rollReverse = new RollReverse(this.sevenEleven);
        private final HoodAim hoodAim = new HoodAim(this.hood);
        private final AvoidDecapitation avoidDecapitation = new AvoidDecapitation(this.hood);
        private final SetShooterVelocity setShooterVelocity = new SetShooterVelocity(this.shooter);
        private final UptakeUp uptakeUp = new UptakeUp(this.uptake, this.turret, this.sevenEleven, this.shooter);
        private final UptakeStop uptakeStop = new UptakeStop(this.uptake, this.shooter);
        // private final UptakeReverse uptakeReverse = new UptakeReverse(uptake);
        // private final ClimbUp climbUp = new ClimbUp(climb);
        // private final ClimbDown climbDown = new ClimbDown(climb);
        // private final ClimbTest climbTest = new ClimbTest(climb);

        public final double currentAngle = this.drivetrain.getState().Pose.getRotation().getDegrees();

        public RobotContainer() {
                this.configureBindings();

                this.drivetrain.setOnPoseResetCallback(pose -> {
                        if (this.oculus.isQuestNavConnected()) {
                                this.oculus.setRobotPose();
                        }
                });

                this.faceAngle.HeadingController.setP(3.1);// 3.54123, 3.1, 3.4123
                this.faceAngle.HeadingController.setI(0);
                this.faceAngle.HeadingController.setD(0);
                this.faceAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

                if (Sim.CURRENT_MODE == Mode.SIM) {
                        this.configureFuelSim();
                }

                this.turret.setDefaultCommand(this.aim);
                this.hood.setDefaultCommand(this.hoodAim);
                this.shooter.setDefaultCommand(this.setShooterVelocity);
                // sevenEleven.setDefaultCommand(rollerPulse);

                NamedCommands.registerCommand("ArmIn", this.intakeArmIn);
                NamedCommands.registerCommand("ArmSlowIn", this.intakeArmInSlow);
                NamedCommands.registerCommand("ArmOut", this.intakeArmOut);
                NamedCommands.registerCommand("IntakeShimmy", new WaitCommand(1.8).andThen(
                                new RepeatCommand(
                                                new IntakeArmMid(this.intakeArm, this.intakeRollers)
                                                                .andThen(new WaitCommand(0.7))
                                                                .andThen(new ParallelCommandGroup(
                                                                                new IntakeArmOut(this.intakeArm),
                                                                                new IntakeRollerShimmy(
                                                                                                this.intakeRollers,
                                                                                                this.intakeArm))
                                                                                .andThen(new WaitCommand(0.7))))));
                NamedCommands.registerCommand("Aim", this.aim);
                NamedCommands.registerCommand("IntakeIn", this.autoIntakeRollerIn);
                NamedCommands.registerCommand("IntakeReverse", this.intakeReverse);
                NamedCommands.registerCommand("IntakeStop", this.intakeRollersStop);
                NamedCommands.registerCommand("Uptake", this.uptakeUp);
                NamedCommands.registerCommand("HoodDown", this.avoidDecapitation);
                NamedCommands.registerCommand("UptakeStop", this.uptakeStop);
                // NamedCommands.registerCommand("ClimbUp", climbUp);
                // NamedCommands.registerCommand("ClimbDown", climbDown);

                this.initializeAutoChooser();
        }

        private double applyDeadband(final double value, final double deadband) {
                if (Math.abs(value) < deadband)
                        return 0.0;
                return value;
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                this.drivetrain.setDefaultCommand(
                                this.drivetrain.applyRequest(() -> this.drive
                                                .withVelocityX(-this.applyDeadband(this.joystick.getLeftY(),
                                                                SwerveConstants.driverDeadband)
                                                                * this.MaxSpeed)
                                                .withVelocityY(-this.applyDeadband(this.joystick.getLeftX(),
                                                                SwerveConstants.driverDeadband)
                                                                * this.MaxSpeed)
                                                .withRotationalRate(
                                                                -this.applyDeadband(this.joystick.getRightX(),
                                                                                SwerveConstants.driverDeadband)
                                                                                * this.MaxAngularRate)));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                // final var idle = new SwerveRequest.Idle();
                // RobotModeTriggers.disabled().whileTrue(
                // drivetrain.applyRequest(() -> idle).ignoringDisable(true)
                // );

                this.joystick.x().whileTrue(
                                this.drivetrain
                                                .applyRequest(() -> this.drive
                                                                .withVelocityX(-this.joystick.getLeftY() * this.MaxSpeed
                                                                                / 10)
                                                                .withVelocityY(-this.joystick.getLeftX() * this.MaxSpeed
                                                                                / 10)
                                                                .withRotationalRate(-this.joystick.getRightX()
                                                                                * this.MaxAngularRate / 10)));

                // joystick.leftBumper().whileTrue(leftDriveToClimb);
                // joystick.rightBumper().whileTrue(rightDriveToClimb);

                // Snapshot angle on press, store it

                this.joystick.b().onTrue(
                                Commands.runOnce(() -> {
                                        this.snappedAngle[0] = Target.getTrenchAngle(
                                                        this.drivetrain.getState().Pose.getTranslation().getX());
                                }));

                this.joystick.b().whileTrue(
                                this.drivetrain.applyRequest(() -> this.faceAngle
                                                .withVelocityX(-this.joystick.getLeftY() * this.MaxSpeed / 2)
                                                .withVelocityY(-this.joystick.getLeftX() * this.MaxSpeed / 2)
                                                .withTargetDirection(this.snappedAngle[0])));

                // joystick.y().onTrue(
                // Commands.runOnce(() -> {
                // snappedAngle[0] = Target.getBumpAngle(
                // drivetrain.getState().Pose.getTranslation().getX()
                // );
                // })
                // );

                // joystick.y().whileTrue(
                // drivetrain.applyRequest(() -> faceAngle
                // .withVelocityX(-joystick.getLeftY() * MaxSpeed / 2)
                // .withVelocityY(-joystick.getLeftX() * MaxSpeed / 2)
                // .withTargetDirection(snappedAngle[0])
                // )
                // );

                this.joystick.y().onTrue(this.drivetrain.applyRequest(() -> this.brake));

                // face desired angle of robot towards the Hub when B is held

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // Reset the field-centric heading on button Y press.
                this.joystick.leftBumper().onTrue(this.drivetrain.runOnce(this.drivetrain::seedFieldCentric));

                this.joystick.a().whileTrue(
                                this.drivetrain.applyRequest(() -> {
                                        final double leftY = -this.joystick.getLeftY();
                                        final double leftX = -this.joystick.getLeftX();

                                        final double magnitude = Math.sqrt(leftX * leftX + leftY * leftY);
                                        Rotation2d targetDirection;

                                        if (magnitude > 0.05) { // Only update rotation when stick is moved
                                                targetDirection = new Rotation2d(this.joystick.getLeftY(),
                                                                this.joystick.getLeftX());

                                                return this.faceAngle
                                                                .withVelocityX(leftY * this.MaxSpeed * 0.54123) // was
                                                                                                                // 0.6
                                                                .withVelocityY(leftX * this.MaxSpeed * 0.54123) // was
                                                                                                                // 0.6
                                                                .withTargetDirection(targetDirection);

                                        } else {
                                                return this.robotStrafe
                                                                .withVelocityX(0) // was 0.6
                                                                .withVelocityY(0);
                                        }

                                }));

                this.joystick.povLeft().whileTrue(this.drivetrain.applyRequest(() -> this.robotStrafe
                                .withVelocityY(0.1 * this.MaxSpeed)
                                .withVelocityX(0)));

                this.joystick.povRight().whileTrue(this.drivetrain.applyRequest(() -> this.robotStrafe
                                .withVelocityY(-0.1 * this.MaxSpeed)
                                .withVelocityX(0)));

                this.joystick.povUp().whileTrue(this.drivetrain.applyRequest(() -> this.robotStrafe
                                .withVelocityX(0.1 * this.MaxSpeed)
                                .withVelocityY(0)));

                this.joystick.povDown().whileTrue(this.drivetrain.applyRequest(() -> this.robotStrafe
                                .withVelocityX(-0.1 * this.MaxSpeed)
                                .withVelocityY(0)));

                this.drivetrain.registerTelemetry(this.logger::telemeterize);

                // --------- Subsystem COMMANDS ---------- // (non swerve subsystem)

                this.joystick.a().onTrue(this.intakeRollersIn);
                this.joystick.a().onFalse(this.intakeRollersStop);

                this.joystick.a().onTrue(this.intakeArmOut);
                this.joystick.a().onFalse(this.intakeRollersStop);

                this.joystick.b().whileTrue(this.avoidDecapitation);
                this.joystick.b().onTrue(this.intakeRollersIn);
                this.joystick.b().onFalse(this.intakeRollersStop);

                this.joystick.rightTrigger().onTrue(this.uptakeUp);
                this.joystick.rightTrigger().onFalse(this.uptakeStop);

                this.joystick.leftTrigger().onTrue(this.forceIntakeArmMid);
                this.joystick.leftTrigger().onFalse(this.intakeArmOut);

                this.joystick.leftStick().onTrue(this.intakeArmIn);

                this.joystick.rightStick().onTrue(this.avoidDecapitation);
                this.joystick.rightStick().onFalse(this.hoodAim);

                // ---------- Buttonboard Commands ------------ //

                this.m_buttonBoard.button(1).onTrue(this.uptakeUp);
                this.m_buttonBoard.button(1).onFalse(this.uptakeStop);

                this.m_buttonBoard.button(2).onTrue(this.intakeReverse);
                this.m_buttonBoard.button(2).onTrue(this.rollReverse);
                this.m_buttonBoard.button(2).onTrue(this.intakeArmOut);
                this.m_buttonBoard.button(2).onFalse(this.intakeRollersStop);
                this.m_buttonBoard.button(2).onFalse(this.uptakeStop);

                this.m_buttonBoard.button(3).onTrue(this.uptakeUp);
                this.m_buttonBoard.button(3).onFalse(this.intakeArmOut);
                this.m_buttonBoard.button(3).onFalse(this.uptakeStop);
                this.m_buttonBoard.button(3).onTrue(this.intakeArmInMid);

                this.m_buttonBoard.button(4).onTrue(this.uptakeUp);
                this.m_buttonBoard.button(4).onFalse(this.uptakeStop);
                this.m_buttonBoard.button(4).onFalse(this.intakeArmOut);
                this.m_buttonBoard.button(4)
                                .whileTrue(new WaitCommand(1.5)
                                                .andThen(new IntakeArmInSlow(this.intakeArm, this.intakeRollers))
                                                .andThen(new WaitCommand(0.75))
                                                .andThen(new IntakeArmOut(this.intakeArm))
                                                .andThen(new WaitCommand(0.75))
                                                .andThen(new ForceIntakeArmMid(this.intakeArm)));

                final Trigger upcomingShiftWarning = new Trigger(
                                () -> ShiftHelpers.isFiveSecBeforeShiftChange(Timer.getMatchTime())
                                                && !ShiftHelpers.currentShiftIsYours());

                final Trigger endingShiftWarning = new Trigger(
                                () -> ShiftHelpers.isTwelveSecBeforeShiftChange(Timer.getMatchTime())
                                                && ShiftHelpers.currentShiftIsYours());

                upcomingShiftWarning.whileTrue(
                                Commands.run(() -> this.joystick.setRumble(RumbleType.kBothRumble, 1.0)));

                endingShiftWarning.whileTrue(
                                Commands.run(() -> this.joystick.setRumble(RumbleType.kBothRumble, 1.0)));

                upcomingShiftWarning.onFalse(
                                Commands.runOnce(() -> this.joystick.setRumble(RumbleType.kBothRumble, 0)));

                endingShiftWarning.onFalse(
                                Commands.run(() -> this.joystick.setRumble(RumbleType.kBothRumble, 0)));
        }

        private void configureFuelSim() {
                final FuelSim instance = FuelSim.getInstance();
                instance.spawnStartingFuel();
                instance.registerRobot(
                                Constants.Sim.fullWidth,
                                Constants.Sim.fullLength,
                                Constants.Sim.fullHeight,
                                () -> this.drivetrain.getState().Pose,
                                () -> this.drivetrain.getState().Speeds);
                // instance.registerIntake(
                // -Constants.Sim.fullLength,
                // Constants.Sim.fullLength / 2,
                // ((-Constants.Sim.fullWidth + 0.5) / 2) + Units.inchesToMeters(7),
                // (-Constants.Sim.fullWidth + 0.5) / 2 ,
                // () -> turretVisSim.canIntake(),
                // () -> turretVisSim.intakeFuel());
                // instance.registerIntake(
                // -Constants.Sim.fullLength / 2,
                // Constants.Sim.fullLength / 2,
                // Constants.Sim.fullWidth / 2,
                // (Constants.Sim.fullWidth / 2) + Units.inchesToMeters(7),
                // () -> turretVisSim.canIntake(),
                // () -> turretVisSim.intakeFuel());
                instance.registerIntake(
                                -Constants.Sim.fullLength / 2,
                                -Constants.Sim.fullLength,
                                -Constants.Sim.fullWidth / 2,
                                Constants.Sim.fullWidth / 2,
                                this.turretVisSim::canIntake,
                                this.turretVisSim::intakeFuel);

                instance.start();

                if (RobotBase.isSimulation()) {
                        this.turret.setDefaultCommand(this.turretVisSim.repeatedlyLaunchFuel(
                                        () -> {
                                                final ShotData shot = TurretCalculator
                                                                .iterativeMovingShotFromFunnelClearance(
                                                                                this.drivetrain.getState().Pose,
                                                                                new ChassisSpeeds(),
                                                                                this.turretVisSim.getTurretTarget(),
                                                                                3);
                                                return shot.getExitVelocity();
                                        },
                                        () -> {
                                                final ShotData shot = TurretCalculator
                                                                .iterativeMovingShotFromFunnelClearance(
                                                                                this.drivetrain.getState().Pose,
                                                                                new ChassisSpeeds(),
                                                                                this.turretVisSim.getTurretTarget(),
                                                                                3);
                                                return shot.getHoodAngle();
                                        },
                                        this.turret));
                }

                SmartDashboard.putData(Commands.runOnce(() -> {
                        FuelSim.getInstance().clearFuel();
                        FuelSim.getInstance().spawnStartingFuel();
                })
                                .withName("Reset Fuel")
                                .ignoringDisable(true));
        }

        public void initializeAutoChooser() {
                this.autoChooser.setDefaultOption("super secret auto",
                                new WaitCommand(3)
                                                .andThen(new ParallelRaceGroup(
                                                                new IntakeArmOut(this.intakeArm),
                                                                new IntakeRollerIn(this.intakeRollers, this.intakeArm),
                                                                new WaitCommand(2)

                                                ))
                                                .andThen(new UptakeUp(this.uptake, this.turret, this.sevenEleven,
                                                                this.shooter)));

                this.autoChooser.addOption("City Boy Left", new ParallelCommandGroup(
                                new WaitCommand(0.01),
                                new SequentialCommandGroup(new CityBoyLeft().cityBoyLeft())));

                this.autoChooser.addOption("City Boy Right", new ParallelCommandGroup(
                                new WaitCommand(0.01),
                                new SequentialCommandGroup(new CityBoyRight().cityBoyRight())));

                this.autoChooser.addOption("MadTown Left", new ParallelCommandGroup(
                                new WaitCommand(0.01),
                                new SequentialCommandGroup(new MadTown().madTownLeft())));

                this.autoChooser.addOption("Orbit Right", new ParallelCommandGroup(
                                new WaitCommand(0.01),
                                new SequentialCommandGroup(new orbit().orbitRight())));
                this.autoChooser.addOption("5m test", new ParallelCommandGroup(
                                new WaitCommand(0.01),
                                new SequentialCommandGroup(new mtest().metertest())));

                SmartDashboard.putData("Auto Selector", this.autoChooser);
        }

        public Command getAutonomousCommand() {
                return this.autoChooser.getSelected();
        }
}
