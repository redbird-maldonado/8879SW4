package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import com.revrobotics.spark.SparkBase.ResetMode;

public class IntakeIOSparkMax implements IntakeIO {
    SparkMax algaeMotor1;
    SparkMax algaeMotor2;
    SparkMax coralIntake;
    SparkMax coralWrist;
    RelativeEncoder wristEncoder;

    public IntakeIOSparkMax() {
        // use actual motor IDs
        algaeMotor1 = new SparkMax(19, MotorType.kBrushless);
        algaeMotor2 = new SparkMax(20, MotorType.kBrushless);
        coralIntake = new SparkMax(17, MotorType.kBrushless);
        coralWrist = new SparkMax(18, MotorType.kBrushless);

        // ask about gear ratios for all motors
        wristEncoder = coralWrist.getEncoder();

        // algaeMotor1.restoreFactoryDefaults();
        // algaeMotor2.restoreFactoryDefaults();

        // algaeMotor1.setSmartCurrentLimit(15);
        // algaeMotor2.setSmartCurrentLimit(15);
        // coralIntake.setSmartCurrentLimit(15);
        // coralWrist.setSmartCurrentLimit(40);

        // coralWrist.getPIDController().setP(0.55);
        // coralWrist.getPIDController().setI(0);
        // coralWrist.getPIDController().setD(0.0);
        // coralWrist.getPIDController().setFF(0.00375);

        // algaeMotor1.setIdleMode(IdleMode.kBrake);
        // algaeMotor2.setIdleMode(IdleMode.kBrake);
        // coralIntake.setIdleMode(IdleMode.kBrake);
        // coralWrist.setIdleMode(IdleMode.kBrake);

        // algaeMotor1.burnFlash();
        // algaeMotor2.burnFlash();
        // coralIntake.burnFlash();
        // coralWrist.burnFlash();
        /////////////////////////////////////////////////////////////////////////////////////
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(15);
        config.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);

        algaeMotor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        config.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(0.55,0,0,0.00375);

        coralWrist.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(15);
        config.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
        algaeMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(15);
        config.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
        coralIntake.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ////////////////////////////////////////////////////////////////////////////////////////
        ///
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.coralWristCurrent = coralWrist.getOutputCurrent();
        inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
        inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
    }

    @Override
    public void setAlgaeVoltage(double voltage) {
        algaeMotor1.setVoltage(voltage);
        algaeMotor2.setVoltage(-voltage);
    }

    @Override
    public void setCoralIntakeVoltage(double voltage) {
        coralIntake.setVoltage(voltage);
    }

    @Override
    public void adjustAngle(double angleRadians) {
        coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
    }

    @Override
    public void wristAngle(double position) {
        // System.out.println("Wrist position: " + getWristPosition());
        coralWrist.getClosedLoopController().setReference(position, SparkMax.ControlType.kPosition);
    }

    @Override
    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    @Override
    public void setWristVoltage(double voltage) {
        // System.out.println("Wrist position: " + getWristPosition());
        coralWrist.set(voltage);
    }
}
