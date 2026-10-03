package frc.robot.configuration.constants.moduleconstants;

import edu.wpi.first.math.util.Units;

public class Dumperconstants {

    public static final int Angulator_MOTOR_CAN_ID = 19;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static double kMinOutput = -1;
    public static double kMaxOutput = 1;  
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static final int kCurrentLimit = 0;
    public static final double kMaxVolts = 12;

    public static final double kLowerLimit = Units.degreesToRotations(0);
    public static final double kUpperLimit = Units.degreesToRotations(40);


     
    //PEDIR
    public static final double kGearRatio = 4;
    public static final boolean kMotorInverted = false;


    

    
}
