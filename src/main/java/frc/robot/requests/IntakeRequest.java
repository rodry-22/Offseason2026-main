package frc.robot.requests;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.configuration.constants.moduleconstants.IntakeConstants;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIO;
import frc.robot.modules.superstructure.modules.IntakeModule.IntakeIO.IntakeInputs;

@RequestFactory
public interface IntakeRequest extends Request<IntakeInputs, IntakeIO> {
    
    public static ModuleColorCode STOP = 
        ModuleColorCode.solid("STOP", Severity.OK, Color.kGray, "System is idle");
    //
    public static ModuleColorCode INTAKING_RPS = 
        ModuleColorCode.solid("INTAKING", Severity.OK, Color.kGreen, "Intake is active");
    public static ModuleColorCode INTAKING_VOLTS = 
        ModuleColorCode.solid("INTAKING", Severity.OK, Color.kGreen, "Intake is active");
    //
    public static ModuleColorCode ANGULATOR_VOLTS = 
        ModuleColorCode.solid("ANGULATOR_MOVING", Severity.OK, Color.kBlue, "Angulator is moving");
    public static ModuleColorCode ANGULATOR_TARGET = 
        ModuleColorCode.solid("ANGULATOR_TARGET", Severity.OK, Color.kPurple, "Angulator is moving to position");
    public static ModuleColorCode ANGULATOR_POSITION = 
        ModuleColorCode.solid("ANGULATOR_POSITION", Severity.OK, Color.kGray, "Angulator is at position"); //checar **

    //

    @CreateCommand(name = "stop")
    public static class Idle implements IntakeRequest {
        @Override
        public ActionStatus apply(IntakeInputs data, IntakeIO actor) {
            actor.stopAll();
            return ActionStatus.of(STOP, "Intake is idle");
        }
    } //creamos comando para detener intake

    @CreateCommand(name = "setRolls")
    public static class SetRolls implements IntakeRequest {

        private double tVolts = 0.0;
        private double tRPS = 0.0;  

        public SetRolls withVoltage(double volts) {
            this.tVolts = volts;
            return this;
        }

        public SetRolls withRPS(double rps) {
            this.tRPS = rps;
            return this;
        }

        @Override
        public ActionStatus apply(IntakeInputs data, IntakeIO actor) {
            if (tVolts != 0.0) {
                actor.setRollsVoltage(tVolts);
                return ActionStatus.of(INTAKING_VOLTS, "Intake is running with volts");

            } else if (tRPS != 0.0) {
                actor.setRollsRPS(tRPS);
                return ActionStatus.of(INTAKING_RPS, "Intake is running with RPS");

            } else {
                actor.stopRolls();
                return ActionStatus.of(STOP, "Rolls are idle");
            }
        }
    }  

    @CreateCommand(name = "setAngulatorPosition")
    public static class SetAngulator implements IntakeRequest {

        private double tPosition = 0.0; 
        private IntakeIO.IntakeMODE mode = IntakeIO.IntakeMODE.kBACK; 
        
        public SetAngulator withPosition(double degrees) {
            this.tPosition = degrees; 
            return this;
        }

         public SetAngulator withMode(IntakeIO.IntakeMODE mode) {
            this.mode = mode;
            return this;
        }

        @Override
        public ActionStatus apply(IntakeInputs data, IntakeIO actor) {
            // Se manda SIEMPRE el setpoint. Antes, un objetivo de 0 (guardado) caia en
            // stopAngulator() y el brazo nunca subia; ademas la tolerancia era 0.
            actor.setAngulatorPosition(tPosition, mode);

            boolean atPosition = MathUtil.isNear(tPosition, data.angulatorPosition, IntakeConstants.toleranceDegrees);

            if (atPosition) {
                return ActionStatus.of(ANGULATOR_POSITION, "Angulator is at position");
            }
            return ActionStatus.of(ANGULATOR_TARGET, "Angulator is moving to position");
        }
    }

    // Voltaje manual del angulador: para medir kG/kS y verificar el sentido del motor.
    @CreateCommand(name = "angulatorVolts")
    public static class SetAngulatorVolts implements IntakeRequest {

        private double volts = 0.0;

        public SetAngulatorVolts withVolts(double volts) {
            this.volts = volts;
            return this;
        }

        @Override
        public ActionStatus apply(IntakeInputs data, IntakeIO actor) {
            actor.setAngulatorVoltage(volts);
            return ActionStatus.of(ANGULATOR_VOLTS, "Angulator manual volts");
        }
    }

}