package frc.robot.configuration.constants.moduleconstants;

public class IntakeConstants {
        
    public static int intakeRatio = 0;
    public static int rollsSparkID = 0;
    public static int angulatorSparkID = 0;



    //sim
    public static double MOIRolls = 0.0; // I =1/3 mL^2
    public static double rollsGearing = 0.0;
    public static int numRollsMotors = 0;

    public static double MOIAngulator = 0.0; // I =1/3 mL^2
    public static double angulatorGearing = 0.0;
    public static int numAngulatorMotors = 0;
    public static double armLength = 0.0; //metros
    public static double minAngle = 0.0;
    public static double maxAngle = 0.0;

    //SJA
    public static double maxVelocity = 0.0; 
    public static double maxAcceleration = 0.0;



    //request
    public static double toleranceDegrees = 0.0; //tolerancia para angulador

    //pid
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kMinOutput = 0.0;
    public static double kMaxOutput = 0.0;  

}
