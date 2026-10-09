package frc.robot.configuration.constants.moduleconstants;

public class Dumperconstants {

    public static final int Angulator_MOTOR_CAN_ID = 19;

    /**
     * true = kP/kI/kD/kG/kS se leen de SmartDashboard ("Dumper/kP", ...) y se aplican en vivo,
     * sin redeploy. Ponerlo en false para competencia (los valores finales se pegan abajo).
     */
    public static final boolean kTuningMode = true;

    // ---------------------------------- MECANICA ----------------------------------
    /** CONFIRMAR: vueltas del mo
     * tor por 1 vuelta de la capucha (reduccion total). */
    public static final double kGearRatio = 4;
    /** Grados de capucha por cada vuelta del NEO. Con esto el encoder del Spark ya lee GRADOS. */
    public static final double kDegreesPerMotorRotation = 360.0 / kGearRatio;
    public static final boolean kMotorInverted = false;

    /** Limites de la capucha en GRADOS (0 = posicion donde se hace el homing / tope mecanico). */
    public static final double kLowerLimitDeg = 0;
    public static final double kUpperLimitDeg = 40;

    // ---------------------------------- CONTROL ----------------------------------
    // PID del Spark: salida en duty cycle (-1..1) por GRADO de error.
    // Valores de PARTIDA, no calibrados.
    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    /** Feedforward en VOLTS, calculado en codigo: kG*cos(angulo + offset) + kS*signo(error). */
    public static final double kG = 0.0;
    public static final double kS = 0.0;
    /** Angulo (grados) al que el centro de masa de la capucha queda horizontal (cos = 1). */

    public static final int kCurrentLimit = 30; // 0 puede dejar el motor sin salida
    public static final double kMaxVolts = 12;

    /** Tolerancia para considerar "en objetivo" (grados). */
    public static final double kToleranceDeg = 1.0;
}