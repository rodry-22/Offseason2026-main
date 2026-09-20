package frc.robot.modules.sumperstructure.modules.Dumpermodule;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;


@Fallback
public interface DumperIO extends IO<DumperIO.DumperInputs>{

    public static class DumperInputs extends Data<DumperInputs> {

         @Unit(value = "Degrees", group = "Dumper")
        public double position = 0;

        @Unit(value = "Degrees", group = "Dumper")
        public double TargetAngle = 0;

        @Unit(value = "Volts", group = "Dumper")
        public double appliedVolts = 0;

        @Unit(value = "RPS", group = "Dumper")
        public double VelocityRPS = 0;

        public double Current = 0;

        @Override
        public DumperInputs snapshot(){
            DumperInputs clone = new DumperInputs();
            clone.TargetAngle = this.TargetAngle;
            clone.appliedVolts = this.appliedVolts;
            clone.VelocityRPS = this.VelocityRPS;
            clone.position = this. position;

            return clone;

        }

        
    }
        // TODO Le cambié esto, pase el enum de la IOKraken acá porque esta IO debería funcionar por si sola y porque 
        // tenía un problema bien raro en la sim todo porque esto estaba en el otro archivo
            public enum DumperMODE{
        kBACK,
        kFRONT
    }

    
    public void setPosition(@Unit(value = "Degrees", group = "Dumper")double angle, DumperMODE mode); 

    public void applyOutput(@Unit(value = "Volts", group = "Dumper") double volts);

    public void resetPosition();

    public void stopAll();
}
