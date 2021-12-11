package frc.robot;

import frc.robot.Logger.SupportedLevels;
import frc.robot.util.geometry.*;

public class Constants {
  public enum Loggers {
    VISION(SupportedLevels.VERBOSE),
    POT(SupportedLevels.DEBUG),
    PID(SupportedLevels.INFO),
    ROBOT(SupportedLevels.WARN),
    TURRET(SupportedLevels.VERBOSE),
    DRIVE(SupportedLevels.INFO),
    SUBSYSTEM(SupportedLevels.INFO),
    BALL_STORED(SupportedLevels.INFO),
    EVENT_WATCHER_THREAD(SupportedLevels.INFO),
    VISION_MANAGER(SupportedLevels.INFO),
    PATH(SupportedLevels.DEBUG),
    OI(SupportedLevels.VERBOSE),
    ROBOT_STATE(SupportedLevels.INFO),
    BALL_DETECTED(SupportedLevels.INFO),
    CAMERA_MANAGER(SupportedLevels.INFO),
    CONFIG(SupportedLevels.INFO),
    STORAGE(SupportedLevels.INFO),
    INTAKE(SupportedLevels.VERBOSE),
    SHOOTER(SupportedLevels.INFO),
    SIMULATEDTALON(SupportedLevels.WARN),
    CLIMBER(SupportedLevels.WARN);

    public SupportedLevels minLevel;

    Loggers(SupportedLevels minLevel) {
    this.minLevel = minLevel;
    }
  }

  // Talon Can IDs
  public static class Talons {
    public static class Drive {
      public static final int leftMaster = 1;
      public static final int leftSlave = 2;
      public static final int rightMaster = 3;
      public static final int rightSlave = 4;
    }

    public static class Arm {
      public static final int rotator = 5;
    }
  }


  // Subsystems
  public static class Drive {
    public static boolean enabled = true;
  
    public static int talonSensorTimeoutMs = 250;
  
    public static double maxSpeedWhenClimbing = .3;
  
    // TODO: determine if this is actually "DRIVE__FORWARD_ACCEL_RAMP_TIME_SECONDS", as it was
    // previously named
    public static double accelSpeed = 1;
    public static double brakeSpeed = 1;


    public static class Encoders {
      public static final double ticksPerWheelRotation = 22000.0;
      public static final double ticksPerMeters = ticksPerWheelRotation * RobotDimensions.RotationsPerMeter;

      // OLD VALUE
      // ticks = (19711 + 19582) / 2
      // distance in feet = 89.5/12
      // ticks per foot = ticks / feet
      private static final double compTicks = (19711.0 + 19582.0) / 2.0;
      private static final double compDistance = 89.5 / 12.0;
    
      public static double compTicksPerFoot = compTicks / compDistance;
      public static final double practiceTicksPerFoot = 1228.615;
    }

    // Diameter of wheel & ticksPerRevolution used to convert meters into encoder ticks
    public static double drivetrainWheelDiameter = 0.1524; // 6 inches in meters
    public static double ticksPerRevolution = 4096;


    public static class AutoPID {
      public static final double p = 4;
      public static final double i = 0.00050;
      public static final double d = 0;
    }
  
    public static class AutoTurnPID {
      public static final double p = 0.2;
      public static final double i = 0;
      public static final double d = 0.015;
      public static final double acceptableError = 0.5;
    }
  }

  public static class Controllers {
    public static final boolean ignore = true;
  
    public static final double joystickDeadband = 0.15;
    public static final double triggerDeadband = 0.15;
  
    public static class Driver {
      public static final int port = 0;
      public static final String name = "Controller (Xbox One For Windows)";
    }
  
    public static class Operator {
      public static final int port = 1;
      public static final String name = "AIRFLO";
    }
  }

  public static class RobotDimensions {
    // tested via trial & error, not measured
    // 22 is too low, 100 is too high
    public static final double driveWheelTrackWidthInches = 50;
    // Based on how this is used, I'm pretty sure this is a corrective factor
    public static final double trackScrubFactor = 1.0469745223;
  
    public static final double driveWheelDiameterInches = 3.938;
    public static final double driveWheelRadiusInches = driveWheelDiameterInches / 2.0;
    public static final double driveWheelTrackRadiusWidthMeters =
      driveWheelTrackWidthInches / 2.0 * 0.0254;
  
    // Offsets from our center point
    public static final Pose2d turretToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));
    public static final Pose2d wheelsToLens =
      new Pose2d(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0.0));

    // Wheel Dimensions
    private static final double WheelDiameter = 6.0; //inches
    private static final double WheelRadius = WheelDiameter/2.0; //inches
    private static final double WheelCircumference = 2 * Misc.pi * WheelRadius; //inches
    private static final double WheelCircumferenceMeters = WheelCircumference * Misc.inchesToMeters; //meters
    private static final double RotationsPerMeter = 1.0/WheelCircumferenceMeters; // meters (rotations per meter)
  }

  public static class Misc {
    public static final double pi = 3.14159;  
    public static final double inchesToMeters = 0.0254; //multiple inches to get meters
  }

  public static class Arm {
    public static final int extenderChannel = 0;
    public static final double ticksPerDegree = 0;
    public static final double shoulderJogSpeed = .1;
    public static final double extenderJogSpeed = .1;
  }
}
