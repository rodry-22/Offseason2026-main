package frc.robot.modules.superstructure.modules.IndexerModule;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.configuration.constants.moduleconstants.IndexerConstants;

public class IndexerIOSpark implements IndexerIO {
    SparkMax rollMotor;
    SparkMax indexMotor;

    SparkMaxConfig rollConfig;
    SparkMaxConfig indexConfig;

    SparkClosedLoopController rollController;
    SparkClosedLoopController indexController;

    public IndexerIOSpark() {
        rollMotor = new SparkMax(IndexerConstants.rollerID, MotorType.kBrushless);
        indexMotor = new SparkMax(IndexerConstants.indexID, MotorType.kBrushless);

        rollConfig = new SparkMaxConfig();
        indexConfig = new SparkMaxConfig();

        rollController = rollMotor.getClosedLoopController();
        indexController = indexMotor.getClosedLoopController();

        //TODO: Gear done, and RPS done. More tuning may be needed.
        rollConfig
            .idleMode(IdleMode.kCoast)
            .encoder.velocityConversionFactor(1/60)
                    .positionConversionFactor(1/IndexerConstants.indexRatio);//TODO:@Units may be doing this, must look into
        indexConfig
            .idleMode(IdleMode.kCoast)
            .encoder.velocityConversionFactor(1/60)
                    .positionConversionFactor(1/IndexerConstants.indexRatio); 
        

        rollMotor.configure(rollConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        indexMotor.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    @Override
	public void updateInputs(IndexerInputs inputs) {
        inputs.rollerVolts = rollMotor.getAppliedOutput() * rollMotor.getBusVoltage();
        inputs.rollerRPS = rollMotor.getEncoder().getVelocity();
        
        inputs.indexVolts = indexMotor.getAppliedOutput() * indexMotor.getBusVoltage();
        inputs.indexRPS = indexMotor.getEncoder().getVelocity();
	}

    @Override
    public void applyRollers(double volts) {
        rollMotor.setVoltage(volts);
    }
    
    @Override
    public void applyIndex(double volts) {
        indexMotor.setVoltage(volts);
    }
    
    @Override
    public void setRollers(double RPS) {
        rollController.setSetpoint(RPS, ControlType.kVelocity);
    }
        
    @Override
    public void setIndex(double RPS) {
        indexController.setSetpoint(RPS, ControlType.kVelocity);
    }
    

    @Override
    public void stopRollers() {
        rollMotor.stopMotor();
    }

    @Override
    public void stopIndex() {
        indexMotor.stopMotor();
    }
    
    @Override
    public void stopAll() {
        indexMotor.stopMotor();
        rollMotor.stopMotor();
    }
    

}

