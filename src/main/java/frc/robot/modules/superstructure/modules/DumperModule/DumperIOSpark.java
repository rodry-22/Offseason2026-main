package frc.robot.modules.superstructure.modules.DumperModule;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.configuration.constants.moduleconstants.Dumperconstants;

public class DumperIOSpark implements DumperIO{

    private final SparkMax angulatorMotor;
    private final RelativeEncoder angulatorEncoder;
    private final SparkClosedLoopController angulatorController; //controlador para angulador

    // Valores "vivos": arrancan con las constantes y, en kTuningMode, se editan desde el Dashboard.
    private double kP = Dumperconstants.kP;
    private double kI = Dumperconstants.kI;
    private double kD = Dumperconstants.kD;
    private double kG = Dumperconstants.kG;
    private double kS = Dumperconstants.kS;

    public DumperIOSpark(){
        angulatorMotor = new SparkMax(Dumperconstants.Angulator_MOTOR_CAN_ID, MotorType.kBrushless);
        angulatorEncoder = angulatorMotor.getEncoder();
        angulatorController = angulatorMotor.getClosedLoopController();

        motorConfig();

        if (Dumperconstants.kTuningMode) {
            SmartDashboard.putNumber("Dumper/kP", kP);
            SmartDashboard.putNumber("Dumper/kI", kI);
            SmartDashboard.putNumber("Dumper/kD", kD);
            SmartDashboard.putNumber("Dumper/kG", kG);
            SmartDashboard.putNumber("Dumper/kS", kS);
        }
    }

    public void motorConfig(){

        var config = new SparkMaxConfig();
        var profiles = config.closedLoop;
        angulatorMotor.setCANTimeout(250);

        try{
            // Slot0 = kBACK, Slot1 = kFRONT. Hoy tienen los mismos valores; si llegan a necesitar
            // ganancias distintas por modo, se separan aqui.
            profiles
            .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            .outputRange(Dumperconstants.kMinOutput, Dumperconstants.kMaxOutput, ClosedLoopSlot.kSlot0)
            .pid(kP, kI, kD, ClosedLoopSlot.kSlot1)
            .outputRange(Dumperconstants.kMinOutput, Dumperconstants.kMaxOutput, ClosedLoopSlot.kSlot1);

            // Sin feedForward del Spark: kV es de velocidad y no aporta en control de posicion.
            // La gravedad (kG) y la friccion (kS) se mandan como arbFeedforward en setPosition().

            config
            .idleMode(IdleMode.kBrake)
            .inverted(Dumperconstants.kMotorInverted)
            .smartCurrentLimit(Dumperconstants.kCurrentLimit)
            .voltageCompensation(Dumperconstants.kMaxVolts);

            // Soft limits en GRADOS (usan las unidades del encoder ya convertidas).
            config
            .softLimit
            .forwardSoftLimit(Dumperconstants.kUpperLimitDeg)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Dumperconstants.kLowerLimitDeg)
            .reverseSoftLimitEnabled(true);

            // Posicion en grados de capucha; velocidad en grados/segundo.
            // OJO: el factor es grados POR VUELTA DE MOTOR (360 / reduccion), no la reduccion.
            config
            .encoder
            .positionConversionFactor(Dumperconstants.kDegreesPerMotorRotation)
            .velocityConversionFactor(Dumperconstants.kDegreesPerMotorRotation / 60.0);

            angulatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        } finally {
            angulatorMotor.setCANTimeout(0);
        }
    }

    /** Lee las ganancias del Dashboard y, si cambiaron, las manda al Spark sin tocar lo demas. */
    private void pollTuning(){
        if (!Dumperconstants.kTuningMode) return;

        double p = SmartDashboard.getNumber("Dumper/kP", kP);
        double i = SmartDashboard.getNumber("Dumper/kI", kI);
        double d = SmartDashboard.getNumber("Dumper/kD", kD);
        kG = SmartDashboard.getNumber("Dumper/kG", kG);
        kS = SmartDashboard.getNumber("Dumper/kS", kS);

        if (p != kP || i != kI || d != kD) {
            kP = p; kI = i; kD = d;
            var cfg = new SparkMaxConfig();
            cfg.closedLoop
                .pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
                .pid(kP, kI, kD, ClosedLoopSlot.kSlot1);
            angulatorMotor.configureAsync(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void updateInputs(DumperInputs inputs){
        pollTuning();

        inputs.position = angulatorEncoder.getPosition();                 // grados
        inputs.VelocityRPM = angulatorEncoder.getVelocity() / 6.0;        // deg/s -> RPM de la capucha
        inputs.appliedVolts = angulatorMotor.getAppliedOutput() * angulatorMotor.getBusVoltage();
        inputs.current = angulatorMotor.getOutputCurrent();
    }

    @Override
    public void setPosition(double angle, DumperMODE mode){
        double target = MathUtil.clamp(angle, Dumperconstants.kLowerLimitDeg, Dumperconstants.kUpperLimitDeg);
        ClosedLoopSlot slot = (mode == DumperMODE.kBACK) ? ClosedLoopSlot.kSlot0 : ClosedLoopSlot.kSlot1;

        double error = target - angulatorEncoder.getPosition();
        double ffVolts =
            kG * Math.cos(Math.toRadians(target + Dumperconstants.kCosOffsetDeg))
            + kS * Math.signum(error);

        angulatorController.setSetpoint(target, ControlType.kPosition, slot, ffVolts);
    }

    @Override
    public void resetPosition(){
        angulatorEncoder.setPosition(Dumperconstants.kLowerLimitDeg);
    }

    @Override
    public void applyOutput(double volts){
        angulatorMotor.setVoltage(volts);
    }

    @Override
    public void stopAll(){
        angulatorMotor.stopMotor();
    }
}