package frc.robot.modules.sumperstructure.modules.flywheelsmodule;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;
import frc.robot.modules.sumperstructure.modules.flywheelsmodule.flywheelsIO.FlyWheelsInputs;
@Fallback
public interface flywheelsIO extends IO<FlyWheelsInputs> {
    
    public static class FlyWheelsInputs extends Data<FlyWheelsInputs> {

    @Unit(value = "Volts", group = "FlyWheel")
    public double appliedVolts = 0;



    @Unit(value = "RPM", group = "FlyWheel")
    public double targetRPM = 0;

    @Unit(value = "RPM", group = "FlyWheel")
    public double velocityRPM = 0;

    public double current = 0;
  }

  public void applyOutput(@Unit(value = "Volts", group = "FlyWheel") double volts);

  public void setSpeed(@Unit(value = "DutyCycle", group = "FlyWheel") double speed);

  public void setTargetRPM(@Unit(value = "RPM", group = "FlyWheel") double rpm);


    
}
