package frc.robot.modules.superstructure.modules.DumperModule;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.configuration.constants.TunerConstants;
import frc.robot.configuration.constants.moduleconstants.Dumperconstants;

public class DumperIOkraken implements DumperIO{

     private TalonFX angulator;
    private TalonFXConfiguration config;
    private TalonFXConfigurator AngulatorConfigurator;
    private MotionMagicExpoVoltage motionRequest;

     public DumperIOkraken(){
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

        public enum DumperMODE{
        kBACK,
        kFRONT
    }

     @Override
    public void setPosition(double angle, DumperMODE mode){
            switch (mode) {
      case kBACK:
        angulator.setControl(
            motionRequest.withPosition(Units.degreesToRotations(angle)).withSlot(0));
        break;

      case kFRONT:
        angulator.setControl(
            motionRequest.withPosition(Units.degreesToRotations(angle)).withSlot(1));
        break;
        }

    }

     @Override
    public void resetPosition(){
        angulator.setPosition(0);
    }

    @Override
    public void applyOutput(double volts){
        angulator.setVoltage(volts);
    }

    @Override
    public void stopAll(){
        angulator.stopMotor();
    }

    @Override 
    public void updateInputs(DumperInputs inputs){

        var rotorPosSignal = angulator.getPosition();
        var rotorPosRotations = rotorPosSignal.getValueAsDouble();

        inputs.position = Units.rotationsToDegrees(rotorPosRotations);

        inputs.timestamp = rotorPosSignal.getTimestamp().getLatency();

        inputs.Current = angulator.getStatorCurrent().getValueAsDouble();

    }


    
}