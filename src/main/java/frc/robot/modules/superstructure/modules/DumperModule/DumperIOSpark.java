package frc.robot.modules.superstructure.modules.DumperModule;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import frc.robot.configuration.constants.moduleconstants.Dumperconstants;

public class DumperIOSpark implements DumperIO{

    private final SparkMax angulatorMotor;
    private final RelativeEncoder angulatorEncoder;
    private final SparkClosedLoopController angulatorController; //controlador para angulador

    public DumperIOSpark(){
        angulatorMotor = new SparkMax(Dumperconstants.Angulator_MOTOR_CAN_ID, MotorType.kBrushless);
        angulatorEncoder = angulatorMotor.getEncoder();
        angulatorController = angulatorMotor.getClosedLoopController();

        motorConfig();
    }

    public void motorConfig(){

        var config = new SparkMaxConfig();
        var profiles = config.closedLoop;
        angulatorMotor.setCANTimeout(250);


        try{
            profiles
            .pid(Dumperconstants.kP, Dumperconstants.kI, Dumperconstants.kD, ClosedLoopSlot.kSlot0)
            .outputRange(Dumperconstants.kMinOutput, Dumperconstants.kMaxOutput, ClosedLoopSlot.kSlot0)

            .pid(Dumperconstants.kP, Dumperconstants.kI, Dumperconstants.kD, ClosedLoopSlot.kSlot1)
            .outputRange(Dumperconstants.kMinOutput, Dumperconstants.kMaxOutput, ClosedLoopSlot.kSlot1);

            profiles.feedForward.kS(Dumperconstants.kS).kV(Dumperconstants.kV).kA(Dumperconstants.kA);

            config
            .idleMode(IdleMode.kBrake)
            .inverted(Dumperconstants.kMotorInverted)
            .smartCurrentLimit(Dumperconstants.kCurrentLimit)
            .voltageCompensation(Dumperconstants.kMaxVolts);

            config
            .softLimit
            .forwardSoftLimit(Dumperconstants.kUpperLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Dumperconstants.kLowerLimit)
            .reverseSoftLimitEnabled(true);

           config
          .encoder
          .positionConversionFactor(Dumperconstants.kGearRatio)
          .velocityConversionFactor(Dumperconstants.kGearRatio);

            angulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        } finally {
      angulatorMotor.setCANTimeout(0);
    }
}

    /* 
    private TalonFX angulator;
    private TalonFXConfiguration config;
    private TalonFXConfigurator AngulatorConfigurator;
    private MotionMagicExpoVoltage motionRequest;

     public DumperIOSpark(){
        angulator = new TalonFX(Dumperconstants.Angulator_MOTOR_CAN_ID, TunerConstants.kCANBus);
        AngulatorConfigurator = angulator.getConfigurator();
        config = new TalonFXConfiguration();

        motionRequest = new MotionMagicExpoVoltage(0);

        configMotion();
    }

     public void configMotion(){
        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.NeutralMode = NeutralModeValue.Brake;

        var limitConfigfs = new CurrentLimitsConfigs();
        limitConfigfs.StatorCurrentLimit = Dumperconstants.CurrentLimit;
        limitConfigfs.StatorCurrentLimitEnable = true;

        var slot0Configs = config.Slot0;

        slot0Configs.kS = 0;
        slot0Configs.kV = 0;
        slot0Configs.kA = 0;
        slot0Configs.kP = 0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;
        slot0Configs.kG = 0;

        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var slot1Configs = config.Slot1;

        slot1Configs.kS = 0;
        slot1Configs.kV = 0;
        slot1Configs.kA = 0;
        slot1Configs.kP = 0;
        slot1Configs.kI = 0;
        slot1Configs.kD = 0;

        AngulatorConfigurator.apply(config);
        AngulatorConfigurator.apply(limitConfigfs);
        AngulatorConfigurator.apply(motorConfigs);    
    }

*/
@Override
public void updateInputs(DumperInputs inputs){
    inputs.VelocityRPM = angulatorEncoder.getVelocity();
    inputs.appliedVolts = angulatorMotor.getAppliedOutput() * angulatorMotor.getBusVoltage();
    inputs.position = Units.rotationsToDegrees(angulatorEncoder.getPosition());
}

     @Override
    public void setPosition(double angle, DumperMODE mode){
        double targetRotations = Units.degreesToRotations(angle);
        ClosedLoopSlot slot = (mode == DumperMODE.kBACK) ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1;

        angulatorController.setSetpoint(targetRotations, ControlType.kPosition, slot);

    }

     @Override
    public void resetPosition(){
        angulatorEncoder.setPosition(0);
    }

    @Override
    public void applyOutput(double volts){
        angulatorMotor.setVoltage(volts);
    }

    @Override
    public void stopAll(){
        angulatorMotor.stopMotor();
    }




    
}