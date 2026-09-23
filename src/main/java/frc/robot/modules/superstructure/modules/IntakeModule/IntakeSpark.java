    package frc.robot.modules.superstructure.modules.IntakeModule;
    import com.revrobotics.PersistMode;
    import com.revrobotics.RelativeEncoder;
    import com.revrobotics.ResetMode;
    import com.revrobotics.spark.SparkBase.ControlType;
    import com.revrobotics.spark.SparkClosedLoopController;
    import com.revrobotics.spark.SparkLowLevel.MotorType;
    import com.revrobotics.spark.SparkMax;
    import com.revrobotics.spark.config.SparkMaxConfig;

    import frc.robot.configuration.constants.moduleconstants.IntakeConstants;

    public class IntakeSpark implements IntakeIO{

        private final SparkMax rollsSpark;
        private final SparkMax angulatorSpark;
        private final RelativeEncoder angulatorEncoder; //encoder para angulador
        private final SparkClosedLoopController rollsController; //controlador para angulador
        private final SparkClosedLoopController angulatorController; //controlador para angulador
        
        public IntakeSpark() {

            //motores
            this.rollsSpark = new SparkMax (IntakeConstants.rollsSparkID, MotorType.kBrushless);
            this.angulatorSpark = new SparkMax (IntakeConstants.angulatorSparkID, MotorType.kBrushless);

            //encoder
            this.angulatorEncoder = angulatorSpark.getEncoder(); //encoder de angulator

            //closed loop controllers
            this.rollsController = rollsSpark.getClosedLoopController(); //para obtener control velocidad de rolls
            this.angulatorController = angulatorSpark.getClosedLoopController(); //obtener control posicion de angulador


            //config de angulator
            var angulatorConfig = new SparkMaxConfig(); //crear config, convertir valores 

            var angulatorProfiles = angulatorConfig.closedLoop;  

            angulatorProfiles.pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
                .outputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);

            angulatorConfig.encoder.positionConversionFactor(360.0); //rotaciones a grados

            //config de rolls
            var rollsConfig = new SparkMaxConfig();

            rollsConfig
                .encoder.velocityConversionFactor(1.0/(IntakeConstants.intakeRatio*60.0));//RPM a RPS

            //aplicar config a motores
            
            angulatorSpark.configure(angulatorConfig,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            rollsSpark.configure(rollsConfig,ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        }

        @Override
        public void updateInputs(IntakeInputs inputs){
            inputs.rollsAppliedVolts = rollsSpark.getAppliedOutput() * rollsSpark.getBusVoltage();
            inputs.rollsRPS = rollsSpark.getEncoder().getVelocity();

            inputs.angulatorAppliedVolts = angulatorSpark.getAppliedOutput() * angulatorSpark.getBusVoltage();
            inputs.angulatorTargetAngle = angulatorController.getSetpoint() *360.0; //convertir a grados
            inputs.angulatorPosition = getAngulatorPosition(); 

        }

        @Override 
        public void setRollsVoltage(double volts) {
            rollsSpark.setVoltage(volts);
        }

        @Override 
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
        public double getAngulatorPosition() {
            return angulatorEncoder.getPosition(); 
        }
        

        @Override
        public void resetAngulator() {
            angulatorEncoder.setPosition(0);
        }

        @Override 
        public void stopAngulator() {
            double currentPosition = angulatorEncoder.getPosition();
            angulatorController.setSetpoint(currentPosition, ControlType.kPosition);
        }

        @Override
        public void stopAll() {
            stopRolls();
            stopAngulator();
        }

    }
