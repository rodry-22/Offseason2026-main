package frc.robot.modules.superstructure.composite;

import com.stzteam.mars.models.multimodules.CompositeIO;

import frc.robot.modules.superstructure.modules.DumperModule.Dumper;

public class SuperstructureIO extends CompositeIO<SuperstructureData>{

    public SuperstructureIO (
        Dumper dumper
        //Intake intake, 
        //Index index
        //FlyWheel flywheelShooter,
        //FlyWheel flywheelsIntake
        ){

            registerChild(dumper);
        }

    @Override 
    public void updateInputs(SuperstructureData inputs){
        
    }

    
}
