package frc.robot.modules.superstructure.modules.IntakeModule;
import frc.robot.configuration.constants.moduleconstants.IntakeConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeSpark implements IntakeIO{

    private final SparkMax rollsSpark;
    private final SparkMax angulatorSpark;
    private final RelativeEncoder intakeEncoder; //encoder para angulador
    private final SparkClosedLoopController rollsController; //controlador para angulador
    private final SparkClosedLoopController angulatorController; //controlador para angulador
    
    public IntakeSpark() {

        this.rollsSpark = new SparkMax (IntakeConstants.rollsSparkID, MotorType.kBrushless);
        this.angulatorSpark = new SparkMax (IntakeConstants.angulatorSparkID, MotorType.kBrushless);

        this.intakeEncoder = angulatorSpark.getEncoder(); //encoder de angulator
        this.rollsController = rollsSpark.getClosedLoopController(); //para obtener control velocidad de rolls

        this.angulatorController = angulatorSpark.getClosedLoopController(); //obtener control posicion de angulador


        var angulatorConfig = new SparkMaxConfig(); //crear config, convertir valores 
        var rollsConfig = new SparkMaxConfig(); 
        

        angulatorConfig
            .encoder.positionConversionFactor(360.0/IntakeConstants.intakeRatio); //Gear ratio de 3:1 *cambiar

        rollsConfig
            .encoder.velocityConversionFactor(1.0/IntakeConstants.intakeRatio*60.0);//RPM a RPS

        angulatorSpark.configure(angulatorConfig,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        rollsSpark.configure(rollsConfig,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void updateInputs(IntakeInputs inputs){
        inputs.rollsAppliedVolts = rollsSpark.getAppliedOutput() * rollsSpark.getBusVoltage();
        inputs.rollsRPS = rollsSpark.getEncoder().getVelocity();

        inputs.angulatorAppliedVolts = angulatorSpark.getAppliedOutput() * angulatorSpark.getBusVoltage();
        inputs.angulatorTargetAngle = angulatorSpark.getEncoder().getPosition(); //preguntar**
        inputs.angulatorPosition = angulatorSpark.getEncoder().getPosition();


    }

    @Override 
    public void setRollsVoltage(double speed) {
        rollsSpark.set(speed);
    }

    public void setRollsRPS(double rps) {
        rollsController.setSetpoint(rps, ControlType.kVelocity); //kVelocity para interpretar como obj de velocidad, no voltaje
    }

    @Override
    public void stopRolls() {
        rollsSpark.set(0);
    }

    @Override 
    public void setAngulatorPosition(double position) {
        angulatorController.setSetpoint(position, ControlType.kPosition); //kPosition para interpretar como obj de position
    }

    @Override
    public void resetAngulator() {
        intakeEncoder.setPosition(0);
    }

    @Override 
    public void stopAngulator() {
        double currentPosition = intakeEncoder.getPosition();
        angulatorController.setSetpoint(currentPosition, ControlType.kPosition);
    }

    @Override
    public void stopAll() {
        rollsSpark.set(0);
        angulatorSpark.set(0);
    }

}
