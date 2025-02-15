package frc.robot;

public class Constants {
    /*Elevator Constants*/
    public static final double kElevatorGearRatio = 0.05; // 20:1 gear ratio on the two elevator motors
    public static final double kElevatorPositionFactor = kElevatorGearRatio * 2.0 * Math.PI;
    public static final double kElevatorVelocityFactor = kElevatorPositionFactor / 60.0;

    public static final double L1_HEIGHT = 3;
    public static final double L2_HEIGHT = 5.5;
    public static final double L3_HEIGHT = 21.5;
    public static final double L4_HEIGHT = 52.5;
}
