package frc.robot.modules.superstructure.modules.IntakeModule;


import java.util.function.Supplier;

import com.stzteam.features.dictionary.Dictionary.CommonTables;
import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.configuration.KeyManager;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIO.IntakeInputs;
import frc.robot.requests.IntakeCommands;
import frc.robot.requests.IntakeRequest;
import frc.robot.requests.IntakeRequestFactory;

public class Intake extends ModularSubsystem<IntakeInputs, IntakeIO> implements IntakeCommands{
    
        public Intake(IntakeIO io) {
        super(
            SubsystemBuilder.<IntakeInputs, IntakeIO>setup()
                .key(KeyManager.INTAKE_KEY)
                .hardware(io, new IntakeInputs())
                .request(new IntakeRequest.Idle())
                .telemetry(new IntakeTelemetry())
            
        );

        setDefaultCommand(runRequest(() -> IntakeRequestFactory.idle()));

    }


    @Override
    public Command setControl(Supplier<IntakeRequest> request) {
        return runRequest(request);
    }

    @Override
    public void absolutePeriodic(IntakeInputs inputs) {}



    public static class IntakeTelemetry extends Telemetry<IntakeInputs> {

        public static final String RPM_ROLLS_KEY = CommonTables.RPM_KEY + "Intake";
        public static final String VOLTS_ROLLS_KEY = CommonTables.VOLTAGE_KEY + "Intake";
        
        public static final String VOLTS_ANGULATOR_KEY = CommonTables.VOLTAGE_KEY + "Intake";
        public static final String ANGULATOR_TARGET_POSITION_KEY = CommonTables.TARGET_KEY + "Intake";
        public static final String ANGULATOR_POSITION_KEY = CommonTables.POSITION_KEY + "Intake";

        @Override
        public void telemeterize(IntakeInputs data) {
           NetworkIO.set(KeyManager.INTAKE_KEY, RPM_ROLLS_KEY, data.rollsRPS);
           NetworkIO.set(KeyManager.INTAKE_KEY, VOLTS_ROLLS_KEY, data.rollsAppliedVolts);

           NetworkIO.set(KeyManager.INTAKE_KEY, VOLTS_ANGULATOR_KEY, data.angulatorAppliedVolts);
           NetworkIO.set(KeyManager.INTAKE_KEY, ANGULATOR_TARGET_POSITION_KEY, data.angulatorTargetAngle);
           NetworkIO.set(KeyManager.INTAKE_KEY, ANGULATOR_POSITION_KEY, data.angulatorPosition);    


        } 

    }


}
