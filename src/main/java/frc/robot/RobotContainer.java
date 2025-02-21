package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.elevator.elevator;
// import frc.robot.subsystems.elevator.elevatorIO; // is dis loggin?
import frc.robot.subsystems.elevator.elevatorIOSparkMax;

// import frc.robot.Constants; // not beeen uzed by us (rite now)
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final elevator elevator;
    private final Intake intake;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        elevator = new elevator(new elevatorIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        configureBindings(); // configurebuttonbindings or configurebindings?
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive
                                                                                                           // forward
                                                                                                           // with
                                                                                                           // negative Y
                                                                                                           // (forward)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* ELEVATOR COMMANDS */
        // L0 state
        Command liftToL0Command = new RunCommand(() -> elevator.setPosition(Constants.L0_HEIGHT), elevator);
        driverController.povDown().onTrue(liftToL0Command);
        // L1 state
        Command liftToL1Command = new RunCommand(() -> elevator.setPosition(Constants.L1_HEIGHT), elevator);
        driverController.leftBumper().onTrue(liftToL1Command);
        // L2 state
        Command liftToL2Command = new RunCommand(() -> elevator.setPosition(Constants.L2_HEIGHT), elevator);
        driverController.rightBumper().onTrue(liftToL2Command);
        // L3 state
        Command liftToL3Command = new RunCommand(() -> elevator.setPosition(Constants.L3_HEIGHT), elevator);
        driverController.leftTrigger().onTrue(liftToL3Command);
        // L4 state
        Command liftToL4Command = new RunCommand(() -> elevator.setPosition(Constants.L4_HEIGHT), elevator);
        driverController.rightTrigger().onTrue(liftToL4Command);

        /* INTAKE COMMANDS */
        // Eject algae
        // NOTE: The original voltage was 12/-12.  We don't want to brown out; just enough voltage to do the job.
        // Setting voltages to 6 for testing.
        Command ejectAlgaeCommand = new StartEndCommand(
                () -> intake.setAlgaeVoltage(6), () -> intake.setAlgaeVoltage(0), intake);
        operatorController.rightBumper().whileTrue(ejectAlgaeCommand); // CHECK IF + IS CW OR CCW.

        Command intakeAlgaeCommand = new StartEndCommand(
                () -> intake.setAlgaeVoltage(-6), () -> intake.setAlgaeVoltage(0), intake);
        operatorController.rightTrigger().whileTrue(intakeAlgaeCommand);

        /* CORAL COMMANDS */
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
