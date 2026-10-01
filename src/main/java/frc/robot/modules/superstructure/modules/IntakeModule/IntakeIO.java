package frc.robot.modules.superstructure.modules.IntakeModule;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

@Fallback
public interface IntakeIO extends IO<IntakeIO.IntakeInputs> {

    public class IntakeInputs extends Data<IntakeInputs> {

        //inputs for rolls
        @Unit(value = "RPS", group = "Intake")
        public double rollsRPS = 0.0; // qué tan rápido están girando las ruedas

        @Unit(value = "Volts", group = "Intake")
        public double rollsAppliedVolts = 0.0;

        //inputs for angulator
        @Unit(value = "Volts", group = "Intake")
        public double angulatorAppliedVolts = 0.0;

        @Unit(value = "Degrees", group = "Intake")
        public double angulatorTargetAngle = 0.0; //position to be   (angulator)

        @Unit(value = "Degrees", group = "Intake")
        public double angulatorPosition = 0.0; //where angulator is

    }


    public void setRollsVoltage(@Unit (value = "Volts" , group = "Intake")double volts);

    public void setRollsRPS(@Unit (value = "RPS" , group = "Intake")double rps);

    public void stopRolls();

    public void setAngulatorPosition(double position);

    public double getAngulatorPosition();

    public void resetAngulator(); //reset encoder pos to 0

    public void stopAngulator();
    
    public void stopAll(); //parar AMBOS

}