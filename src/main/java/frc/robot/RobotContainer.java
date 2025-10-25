// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.AlgaeRollersConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.CoralRollersConstants;
import frc.robot.Constants.AlgaePivotConstants.AlgaePivotPositions;
import frc.robot.Constants.AlgaeRollersConstants.AlgaeRollerModes;
import frc.robot.Constants.CoralRollersConstants.CoralRollerModes;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollers;
import lombok.Getter;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final Climber climber = new Climber(new SparkMax(ClimberConstants.id, MotorType.kBrushless), true);

        public final CoralRollers coralRollers = new CoralRollers(
                        new SparkMax(CoralRollersConstants.id, MotorType.kBrushless), true);

        public final AlgaeRollers algaeRollers = new AlgaeRollers(
                        new SparkMax(AlgaeRollersConstants.id, MotorType.kBrushless),
                        true);

        public final AlgaePivot algaePivot = new AlgaePivot(new SparkMax(AlgaePivotConstants.id, MotorType.kBrushless),
                        true);

        public final Trigger climberForward = operator.rightBumper();
        public final Trigger climberBack = operator.leftBumper();

        private Trigger zeroDrivebase = driver.y();

        public final Trigger coralRollersTrigger = operator.y();

        public final Trigger algaeIntake = operator.x();
        public final Trigger algaeOuttake = operator.b();

        public final Trigger algaePivotUp = operator.leftTrigger();
        public final Trigger algaePivotDown = operator.rightTrigger();

        private @Getter Trigger hasAlgae = new Trigger(() -> algaeRollers.hasAlgae());

        public final Trigger algaePivotStow = operator.povLeft();
        public final Trigger algaePivotFeed = operator.povUp();
        public final Trigger algaePivotProcessor = operator.povDown();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {
                // Path Planner reccomends that construction of their namedcommands happens
                // before anything else in robot container
                setCommandMappings();

                // Regular Two Controllers, comment out whatever not wanted
                configureOperatorControls();
                configureDriverControls();

                // One Controller
                // configureSingleControls();

                autoChooser = AutoBuilder.buildAutoChooser();

                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureDriverControls() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(deadzone(-driver.getLeftY()) * MaxSpeed) // Drive
                                                                                                        // forward
                                                                                                        // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(deadzone(-driver.getLeftX()) * MaxSpeed) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
                                                .withRotationalRate(deadzone(-driver.getRightX()) * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                // negative X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driver.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                // reset the field-centric heading on left bumper press
                driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

                zeroDrivebase.onTrue(new InstantCommand(() -> drivetrain.resetGyro())
                                .withName("Zero Drivebase"));

        }

        private void configureOperatorControls() {
                climberForward.whileTrue(new RunCommand(() -> climber.run(.2), climber));
                climberBack.whileTrue(new RunCommand(() -> climber.run(-0.2), climber));

                coralRollersTrigger.whileTrue(
                                coralRollers.runRollersCommand(CoralRollerModes.OUTTAKE)
                                                .withName("Operator Coral Outtake"));

                algaeOuttake.whileTrue(
                                algaeRollers.runRollersCommand(AlgaeRollerModes.OUTTAKE)
                                                .withName("Operator Algae Outtake"));
                algaeIntake.whileTrue(algaeRollers.runRollersCommand(AlgaeRollerModes.INTAKE)
                                .withName("Operator Algae Intake"));

                algaePivotUp.whileTrue(algaePivot.run(-0.15));
                algaePivotDown.whileTrue(algaePivot.run(0.2));

                algaePivotFeed.onTrue(algaePivot.runAlgaePivotCommand(AlgaePivotPositions.FEED));
                algaePivotProcessor.onTrue(algaePivot.runAlgaePivotCommand(AlgaePivotPositions.PROCESSOR));
                algaePivotStow.onTrue(algaePivot.runAlgaePivotCommand(AlgaePivotPositions.STOW)
                                .withName("Operator Algae Pivot Stow"));
        }

        public void periodic() {
                SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
        }

        private void configureSingleControls() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(deadzone(-driver.getLeftY()) * MaxSpeed) // Drive
                                                                                                        // forward
                                                                                                        // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(deadzone(-driver.getLeftX()) * MaxSpeed) // Drive left
                                                                                                        // with negative
                                                                                                        // X (left)
                                                .withRotationalRate(deadzone(-driver.getRightX()) * MaxAngularRate) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                // negative X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driver.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

                driver.rightBumper().whileTrue(new RunCommand(() -> climber.run(.2), climber));
                driver.leftBumper().whileTrue(new RunCommand(() -> climber.run(-0.2), climber));

                driver.y().whileTrue(
                                coralRollers.runRollersCommand(CoralRollerModes.OUTTAKE)
                                                .withName("Single Control Coral Outtake"));

                driver.b().whileTrue(
                                algaeRollers.runRollersCommand(AlgaeRollerModes.OUTTAKE)
                                                .withName("Single Control Algae Outtake"));
                driver.x().whileTrue(algaeRollers.runRollersCommand(AlgaeRollerModes.INTAKE)
                                .withName("Single Control Algae Intake"));

                // driver.povUp().whileTrue(algaePivot.run(0.2));
                // driver.povDown().whileTrue(algaePivot.run(-0.2));

                // algaePivotFeed.onTrue(algaePivot.runAlgaePivotCommand(AlgaePivotPositions.FEED));
                // algaePivotProcessor.onTrue(algaePivot.runAlgaePivotCommand(AlgaePivotPositions.PROCESSOR));
                // algaePivotStow.onTrue(algaePivot.runAlgaePivotCommand(AlgaePivotPositions.STOW)
                // .withName("Single Control Algae Pivot Stow"));

        }

        public void onInitialize() {
                algaePivot.runAlgaePivotCommand(AlgaePivotPositions.STOW);
                coralRollers.setDefaultCommand(new RunCommand(() -> coralRollers.stop(), coralRollers));

                // if (hasAlgae.getAsBoolean()) {
                // algaeRollers.setDefaultCommand(algaeRollers.run(.05));
                // } else {
                // algaeRollers.setDefaultCommand(new RunCommand(() -> algaeRollers.stop(),
                // algaeRollers));
                // }

                algaeRollers.setDefaultCommand(new RunCommand(() -> algaeRollers.stop(), algaeRollers));

                climber.setDefaultCommand(new RunCommand(() -> climber.stop(), climber));
                algaeRollers.setDefaultCommand(new RunCommand(() -> algaeRollers.stop(), algaeRollers));
                algaePivot.setDefaultCommand(new RunCommand(() -> algaePivot.stop(), algaePivot));
        }

        /**
         * Sets a Deazone
         * Make a linear function with deadson at 0 and 1 at 1.
         * Then need to have this work on both positive and negative.
         * 
         * @param num
         * @return
         */
        double deadzone = .05;

        public double deadzone(double num) {
                if (Math.abs(num) > deadzone) {
                        double w = 1.0 / (1.0 - deadzone);
                        double b = w * deadzone;
                        return (w * Math.abs(num) - b) * (num / Math.abs(num));
                } else {
                        return 0;
                }
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setCommandMappings() {
                NamedCommands.registerCommand("RunCoralRollers",
                                new ParallelRaceGroup(coralRollers.runRollersCommand(CoralRollerModes.OUTTAKE),
                                                new WaitCommand(1)));
        }

}
