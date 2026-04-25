// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANdi;
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
import frc.robot.commands.intakearm.ForceIntakeArmMid;
import frc.robot.commands.intakearm.IntakeArmIn;
import frc.robot.commands.intakearm.IntakeArmInMid;
import frc.robot.commands.intakearm.IntakeArmInSlow;
import frc.robot.commands.intakearm.IntakeArmMid;
import frc.robot.commands.intakearm.IntakeArmOut;
import frc.robot.commands.intakeroller.AutoIntakeRollerIn;
import frc.robot.commands.intakeroller.IntakeReverse;
import frc.robot.commands.intakeroller.IntakeRollerIn;
import frc.robot.commands.intakeroller.IntakeRollerShimmy;
import frc.robot.commands.intakeroller.IntakeRollerStop;
import frc.robot.commands.seveneleven.RollReverse;
import frc.robot.commands.seveneleven.RollStop;
import frc.robot.commands.shooter.SetShooterVelocity;
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
import frc.robot.subsystems.turret.TrajectoryCalculator;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretVisSim;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShiftHelpers;
import frc.robot.utils.Target;

public class RobotContainer {
    // =========================
    // DRIVETRAIN / CORE CONTROL
    // =========================

    // kSpeedAt12Volts desired top speed
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);

    // 3/4 of a rotation per second max angular velocity
    private final double maxAngularRate =
            Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond);

    final Rotation2d[] snappedAngle = {new Rotation2d()};

    // =========================
    // SWERVE REQUESTS
    // =========================

    // Using a 10% deadband
    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(this.maxSpeed * 0.05)
                    .withRotationalDeadband(this.maxAngularRate * 0.05)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final SwerveRequest.FieldCentricFacingAngle faceAngle =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position)
                    .withDeadband(this.maxSpeed * 0.05)
                    .withRotationalDeadband(this.maxAngularRate * 0.05);

    private final SwerveRequest.FieldCentric robotStrafe =
            new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // =========================
    // UTIL / TELEMETRY
    // =========================

    private final Telemetry logger = new Telemetry(this.maxSpeed);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // =========================
    // INPUT DEVICES
    // =========================

    private final CommandXboxController joystick =
            new CommandXboxController(InputConstants.DRIVER_CONTROLLER_PORT_0);

    private final CommandGenericHID buttonBoard =
            new CommandGenericHID(InputConstants.DRIVER_CONTROLLER_PORT_1);

    // =========================
    // HARDWARE / SUBSYSTEMS
    // =========================

    private final CANdi candi =
            new CANdi(Constants.CanIdCanivore.INTAKE_CANDI, Constants.CanIdCanivore.CARNIVORE);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Oculus oculus = new Oculus();

    // private final Vision vision = new Vision();

    private final Turret turret = new Turret(this.drivetrain);

    private final TurretVisSim turretVisSim =
            new TurretVisSim(() -> new Pose3d(this.drivetrain.getState().Pose), this.turret);

    private final IntakeRoller intakeRollers = new IntakeRoller();

    private final SevenEleven sevenEleven = new SevenEleven();

    private final IntakeArm intakeArm = new IntakeArm(this.candi);

    private final Hood hood = new Hood(this.candi);

    private final Shooter shooter = new Shooter();

    private final Uptake uptake = new Uptake();

    // =========================
    // COMMANDS
    // =========================

    private final Aim aim = new Aim(this.turret, this.drivetrain);

    private final IntakeRollerIn intakeRollersIn =
            new IntakeRollerIn(this.intakeRollers, this.intakeArm);

    private final AutoIntakeRollerIn autoIntakeRollerIn =
            new AutoIntakeRollerIn(this.intakeRollers, this.intakeArm);

    private final IntakeRollerStop intakeRollersStop = new IntakeRollerStop(this.intakeRollers);

    private final IntakeReverse intakeReverse =
            new IntakeReverse(this.intakeRollers, this.intakeArm);

    private final IntakeArmIn intakeArmIn = new IntakeArmIn(this.intakeArm, this.intakeRollers);

    private final IntakeArmInSlow intakeArmInSlow =
            new IntakeArmInSlow(this.intakeArm, this.intakeRollers);

    private final IntakeArmInMid intakeArmInMid =
            new IntakeArmInMid(this.intakeArm, this.intakeRollers);

    private final IntakeArmOut intakeArmOut = new IntakeArmOut(this.intakeArm);

    private final ForceIntakeArmMid forceIntakeArmMid = new ForceIntakeArmMid(this.intakeArm);

    private final RollReverse rollReverse = new RollReverse(this.sevenEleven);

    private final RollStop rollStop = new RollStop(this.sevenEleven);

    private final HoodAim hoodAim = new HoodAim(this.hood);

    private final AvoidDecapitation avoidDecapitation = new AvoidDecapitation(this.hood);

    private final SetShooterVelocity setShooterVelocity = new SetShooterVelocity(this.shooter);

    private final UptakeUp uptakeUp =
            new UptakeUp(this.uptake, this.turret, this.sevenEleven, this.shooter);

    private final UptakeStop uptakeStop = new UptakeStop(this.uptake, this.shooter);

    // =========================
    // STATE
    // =========================

    public final double currentAngle = this.drivetrain.getState().Pose.getRotation().getDegrees();

    public RobotContainer() {
        this.configureBindings();
        this.candi.optimizeBusUtilization();

        this.drivetrain.setOnPoseResetCallback(
                pose -> {
                    if (this.oculus.isQuestNavConnected()) {
                        this.oculus.setRobotPose();
                    }
                });

        this.faceAngle.HeadingController.setP(3.4123); // 3.54123, 3.1, 3.4123
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
        NamedCommands.registerCommand(
                "IntakeShimmy",
                new WaitCommand(1.8)
                        .andThen(
                                new RepeatCommand(
                                        new IntakeArmMid(this.intakeArm, this.intakeRollers)
                                                .andThen(new WaitCommand(0.7))
                                                .andThen(
                                                        new ParallelCommandGroup(
                                                                        new IntakeArmOut(
                                                                                this.intakeArm),
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
        this.initializeAutoChooser();
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }

    private double applyDeadband(final double value, final double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        return value;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        this.drivetrain.setDefaultCommand(
                this.drivetrain.applyRequest(
                        () ->
                                this.drive
                                        .withVelocityX(
                                                -this.applyDeadband(
                                                                this.joystick.getLeftY(),
                                                                SwerveConstants.DRIVER_DEADBAND)
                                                        * this.maxSpeed)
                                        .withVelocityY(
                                                -this.applyDeadband(
                                                                this.joystick.getLeftX(),
                                                                SwerveConstants.DRIVER_DEADBAND)
                                                        * this.maxSpeed)
                                        .withRotationalRate(
                                                -this.applyDeadband(
                                                                this.joystick.getRightX(),
                                                                SwerveConstants.DRIVER_DEADBAND)
                                                        * this.maxAngularRate)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        // drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        this.joystick
                .x()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () ->
                                        this.drive
                                                .withVelocityX(
                                                        -this.joystick.getLeftY()
                                                                * this.maxSpeed
                                                                / 10)
                                                .withVelocityY(
                                                        -this.joystick.getLeftX()
                                                                * this.maxSpeed
                                                                / 10)
                                                .withRotationalRate(
                                                        -this.joystick.getRightX()
                                                                * this.maxAngularRate
                                                                / 10)));

        // joystick.leftBumper().whileTrue(leftDriveToClimb);
        // joystick.rightBumper().whileTrue(rightDriveToClimb);

        // Snapshot angle on press, store it

        this.joystick
                .b()
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        this.snappedAngle[0] =
                                                Target.getTrenchAngle(
                                                        this.drivetrain
                                                                .getState()
                                                                .Pose
                                                                .getTranslation()
                                                                .getX())));

        this.joystick
                .b()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () ->
                                        this.faceAngle
                                                .withVelocityX(
                                                        -this.joystick.getLeftY()
                                                                * this.maxSpeed
                                                                / 2)
                                                .withVelocityY(
                                                        -this.joystick.getLeftX()
                                                                * this.maxSpeed
                                                                / 2)
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
        this.joystick
                .leftBumper()
                .onTrue(this.drivetrain.runOnce(this.drivetrain::seedFieldCentric));

        this.joystick
                .a()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () -> {
                                    final double leftY = -this.joystick.getLeftY();
                                    final double leftX = -this.joystick.getLeftX();

                                    final double magnitude =
                                            Math.sqrt(leftX * leftX + leftY * leftY);
                                    Rotation2d targetDirection;

                                    if (magnitude
                                            > 0.05) { // Only update rotation when stick is moved
                                        targetDirection =
                                                new Rotation2d(
                                                        this.joystick.getLeftY(),
                                                        this.joystick.getLeftX());

                                        return this.faceAngle
                                                .withVelocityX(
                                                        leftY * this.maxSpeed * 0.54123) // was
                                                // 0.6
                                                .withVelocityY(
                                                        leftX * this.maxSpeed * 0.54123) // was
                                                // 0.6
                                                .withTargetDirection(targetDirection);

                                    } else {
                                        return this.robotStrafe
                                                .withVelocityX(0) // was 0.6
                                                .withVelocityY(0);
                                    }
                                }));

        this.joystick
                .povLeft()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () ->
                                        this.robotStrafe
                                                .withVelocityY(0.1 * this.maxSpeed)
                                                .withVelocityX(0)));

        this.joystick
                .povRight()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () ->
                                        this.robotStrafe
                                                .withVelocityY(-0.1 * this.maxSpeed)
                                                .withVelocityX(0)));

        this.joystick
                .povUp()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () ->
                                        this.robotStrafe
                                                .withVelocityX(0.1 * this.maxSpeed)
                                                .withVelocityY(0)));

        this.joystick
                .povDown()
                .whileTrue(
                        this.drivetrain.applyRequest(
                                () ->
                                        this.robotStrafe
                                                .withVelocityX(-0.1 * this.maxSpeed)
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

        // TODO make a shot that is independent of pose

        this.joystick.leftStick().onTrue(this.intakeArmIn);

        this.joystick.rightStick().onTrue(this.avoidDecapitation);
        this.joystick.rightStick().onFalse(this.hoodAim);

        // joystick.button(8).onTrue(manualReset);
        // joystick.button(8).onFalse(hoodAim);

        // ---------- Buttonboard Commands ------------ //

        this.buttonBoard.button(1).onTrue(this.uptakeUp);
        this.buttonBoard.button(1).onFalse(this.uptakeStop);

        this.buttonBoard.button(2).onTrue(this.intakeReverse);
        this.buttonBoard.button(2).onTrue(this.intakeArmOut);
        this.buttonBoard.button(2).onTrue(this.rollReverse);
        this.buttonBoard.button(2).onFalse(this.rollStop);

        this.buttonBoard.button(3).onTrue(this.uptakeUp);
        this.buttonBoard.button(3).onFalse(this.intakeArmOut);
        this.buttonBoard.button(3).onFalse(this.uptakeStop);
        this.buttonBoard.button(3).onTrue(this.intakeArmInMid);

        this.buttonBoard.button(4).onTrue(this.uptakeUp);
        this.buttonBoard.button(4).onFalse(this.uptakeStop);
        this.buttonBoard.button(4).onFalse(this.intakeArmOut);
        this.buttonBoard
                .button(4)
                .whileTrue(
                        new WaitCommand(1.5)
                                .andThen(new IntakeArmInSlow(this.intakeArm, this.intakeRollers))
                                .andThen(new WaitCommand(0.75))
                                .andThen(new IntakeArmOut(this.intakeArm))
                                .andThen(new WaitCommand(0.75))
                                .andThen(new ForceIntakeArmMid(this.intakeArm)));
        final Trigger upcomingShiftWarning =
                new Trigger(
                        () ->
                                ShiftHelpers.isFiveSecBeforeShiftChange(Timer.getMatchTime())
                                        && !ShiftHelpers.currentShiftIsYours());

        final Trigger endingShiftWarning =
                new Trigger(
                        () ->
                                ShiftHelpers.isTwelveSecBeforeShiftChange(Timer.getMatchTime())
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
                Constants.Sim.FULL_WIDTH,
                Constants.Sim.FULL_LENGTH,
                Constants.Sim.FULL_HEIGHT,
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
                -Constants.Sim.FULL_LENGTH / 2,
                -Constants.Sim.FULL_LENGTH,
                -Constants.Sim.FULL_WIDTH / 2,
                Constants.Sim.FULL_WIDTH / 2,
                this.turretVisSim::canIntake,
                this.turretVisSim::intakeFuel);

        instance.start();

        if (RobotBase.isSimulation()) {
            this.turret.setDefaultCommand(
                    this.turretVisSim.repeatedlyLaunchFuel(
                            () -> {
                                final ShotData shot =
                                        TrajectoryCalculator.iterativeMovingShotFromFunnelClearance(
                                                this.drivetrain.getState().Pose,
                                                new ChassisSpeeds(),
                                                this.turretVisSim.getTurretTarget(),
                                                3);
                                return shot.getExitVelocity();
                            },
                            () -> {
                                final ShotData shot =
                                        TrajectoryCalculator.iterativeMovingShotFromFunnelClearance(
                                                this.drivetrain.getState().Pose,
                                                new ChassisSpeeds(),
                                                this.turretVisSim.getTurretTarget(),
                                                3);
                                return shot.getHoodAngle();
                            },
                            this.turret));
        }

        SmartDashboard.putData(
                Commands.runOnce(
                                () -> {
                                    FuelSim.getInstance().clearFuel();
                                    FuelSim.getInstance().spawnStartingFuel();
                                })
                        .withName("Reset Fuel")
                        .ignoringDisable(true));
    }

    public void initializeAutoChooser() {
        this.autoChooser.setDefaultOption("super secret auto", new WaitCommand(5));

        this.autoChooser.addOption(
                "City Boy Left",
                new ParallelCommandGroup(
                        new WaitCommand(0.01),
                        new SequentialCommandGroup(new CityBoyLeft().cityBoyLeft())));

        this.autoChooser.addOption(
                "City Boy Right",
                new ParallelCommandGroup(
                        new WaitCommand(0.01),
                        new SequentialCommandGroup(new CityBoyRight().cityBoyRight())));

        this.autoChooser.addOption(
                "MadTown Left",
                new ParallelCommandGroup(
                        new WaitCommand(0.01),
                        new SequentialCommandGroup(new MadTown().madTownLeft())));

        this.autoChooser.addOption(
                "Orbit Right",
                new ParallelCommandGroup(
                        new WaitCommand(0.01),
                        new SequentialCommandGroup(new orbit().orbitRight())));

        this.autoChooser.addOption(
                "Orbit Right Delay",
                new SequentialCommandGroup(new WaitCommand(4).andThen(new orbit().orbitRight())));

        this.autoChooser.addOption(
                "5m test",
                new ParallelCommandGroup(
                        new WaitCommand(0.01),
                        new SequentialCommandGroup(new mtest().metertest())));

        SmartDashboard.putData("Auto Selector", this.autoChooser);
    }
}
