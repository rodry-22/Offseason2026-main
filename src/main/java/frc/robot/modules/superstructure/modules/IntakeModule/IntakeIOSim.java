package frc.robot.modules.superstructure.modules.IntakeModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.configuration.constants.moduleconstants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

    //rolls
    private final FlywheelSim rollsSim;
    private final DCMotor rollsGearbox;
    private double rollsAppliedVolts = 0.0;

    //angulator
    private final SingleJointedArmSim angulatorSim;
    private final ProfiledPIDController angulatorController;
    private double angulatorAppliedVolts = 0.0;
    private double angulatorTargetAngle = 0.0;
    private final DCMotor angulatorGearbox;

    private boolean isClosedLoop = false;


    public IntakeIOSim() {

        //rolls
        double moiRolls = IntakeConstants.MOIRolls; // I =1/3 mL^2
        double rollsGearing = IntakeConstants.rollsGearing;

        this.rollsGearbox = DCMotor.getNEO(IntakeConstants.numRollsMotors); //cambiar # motores usados

        var rollsPlant = LinearSystemId.createFlywheelSystem(rollsGearbox, moiRolls, rollsGearing);
        this.rollsSim = new FlywheelSim(rollsPlant, rollsGearbox, rollsGearing);


        //angulator
        double moiAngulator = IntakeConstants.MOIAngulator; // I =1/3 mL^2
        double angulatorGearing = IntakeConstants.angulatorGearing;
        this.angulatorGearbox = DCMotor.getNEO(IntakeConstants.numAngulatorMotors); //cambiar # motores usados

        this.angulatorSim = new SingleJointedArmSim(angulatorGearbox, 
            angulatorGearing, 
            moiAngulator, 
            IntakeConstants.armLength, 
            IntakeConstants.minAngle,           
            IntakeConstants.maxAngle, 
            false, 
            0.0);

        this.angulatorController = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, 
            new TrapezoidProfile.Constraints(IntakeConstants.maxVelocity, IntakeConstants.maxAcceleration)); //tunear valores
        
    }  

    @Override 
    public void updateInputs(IntakeInputs inputs) {

        //rolls
        rollsAppliedVolts = MathUtil.clamp(rollsAppliedVolts, -12.0, 12.0);
        rollsSim.setInputVoltage(rollsAppliedVolts); //"voltaje actual"
        rollsSim.update(0.02); // aplicar volataje actual durante 20 ms (actualizar simulacion cada 20ms)

        inputs.rollsAppliedVolts = rollsAppliedVolts;
        inputs.rollsRPS = rollsSim.getAngularVelocityRPM()/60.0; //RPM to RPS

        
        //angulator
        if (isClosedLoop) {
            double currentPosRotations = Units.radiansToRotations(angulatorSim.getAngleRads());
            angulatorAppliedVolts = angulatorController.calculate(currentPosRotations);
        }

        angulatorAppliedVolts = MathUtil.clamp(angulatorAppliedVolts, -12.0, 12.0);
        angulatorSim.setInputVoltage(angulatorAppliedVolts); 
        angulatorSim.update(0.02); 

        inputs.angulatorAppliedVolts = angulatorAppliedVolts;
        inputs.angulatorTargetAngle = angulatorTargetAngle;
        inputs.angulatorPosition = getAngulatorPosition();
    }

    @Override 
    public void setRollsVoltage(double volts) {
        isClosedLoop = false;
        rollsAppliedVolts = volts;
    }

    @Override
    public void setRollsRPS(double rps) {
        // no sé :)
    }

    @Override 
    public void stopRolls() {
        rollsAppliedVolts = 0.0;
    }

    @Override 
    public void setAngulatorPosition(double position) {
        isClosedLoop = true;
        this.angulatorTargetAngle = position;
        angulatorController.setGoal(Units.degreesToRotations(position));
    }
    
    @Override
    public double getAngulatorPosition() {
        return Units.radiansToDegrees(angulatorSim.getAngleRads());
    }

    @Override
    public void resetAngulator() {
        isClosedLoop = false;
        angulatorSim.setState(0, 0.0);  //pos 0, vel 0
    }

    @Override 
    public void stopAngulator(){
        isClosedLoop = false;
        angulatorAppliedVolts = 0.0;
    }


    @Override 
    public void stopAll() {
        stopRolls();
        stopAngulator();
    }



}
