package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkBaseConfig.Inverted; // added bc weren't sure how to invert
// import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
// import com.google.flatbuffers.Constants; // what iz dis? from penn state
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants;

public class elevatorIOSparkMax implements elevatorIO {
  private final SparkMax leadMotor;
  public final SparkMax followerMotor;
  private final RelativeEncoder leadEncoder;
  private final RelativeEncoder followerEncoder;
  public final SparkClosedLoopController leadPIDController;

  // Constructor
  public elevatorIOSparkMax() {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new SparkMax(15, MotorType.kBrushless); // maybe don't need the deviceId?
    followerMotor = new SparkMax(16, MotorType.kBrushless);

    leadEncoder = leadMotor.getEncoder(); // only need this bc other motor follows
    followerEncoder = followerMotor.getEncoder();
    
    this.leadPIDController = this.leadMotor.getClosedLoopController();

    SparkMaxConfig leadConfig = new SparkMaxConfig();
      leadConfig
        .inverted(true) //Inverting lead motor because positive makes elevator go down
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        // .voltageCompensation(12); // limit controller max V; can adjust if nec.
        .encoder.positionConversionFactor(Constants.kElevatorPositionFactor)
                .velocityConversionFactor(Constants.kElevatorVelocityFactor);

        //.encoder.positionConversionFactor(360d * ABSOLUTE_DEGREES_PER_RELATIVE_DEGREES / 762.183 * METERS_ASCENDED_PER_ROTATION)
    SparkMaxConfig followConfig = new SparkMaxConfig();
      followConfig
        // .inverted(true)
        .follow(15, false)
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);


    leadConfig.closedLoop.pidf(.027, 0, 0, .0085) // this year all the constants are on one line
              .outputRange(-.8, .8);
    // leadMotor.getPIDController().setP(0.027);
    // leadMotor.getPIDController().setI(0);
    // leadMotor.getPIDController().setD(0);
    // leadMotor.getPIDController().setFF(0.0085);

    // leadMotor.SetIdleMode(IdleMode.kBrake);
    // followerMotor.SetIdleMode(IdleMode.kBrake);

    // SparkBaseConfig.IdleMode kBrake;    

    // leadMotor.burnFlash();
    // followerMotor.burnFlash();
    leadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    followerMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leadEncoder.setPosition(0);

    followerEncoder.setPosition(0);
  }

  // IF we want to add limit switches, this is the stuff
  // @Override 
  // public void periodic() {
  //   System.out.println("L ls: " + this.getLeftLimitSwitch() + ", R: " + this.getRightLimitSwitch());
  //   rightHeightRelPub.setDouble(this.getRightRelativePosition());
  //   leftHeightRelPub.setDouble(this.getLeftRelativePosition());
  // }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return leadEncoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the velocity from the encoder
    return leadEncoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specified position
    leadEncoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}