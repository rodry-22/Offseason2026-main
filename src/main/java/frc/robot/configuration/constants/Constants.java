package frc.robot.configuration.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {

    public static final double FLYWHEEL_TOLERANCE = 34.72;

    // ---------------------------------- CAMPO / HUB ----------------------------------
    // VERIFICAR contra el layout oficial 2026 (AprilTagFieldLayout / manual del juego) antes de usar.
    // Un error aqui se convierte directo en error de distancia y de angulo.
    public static final double FIELD_LENGTH_METERS = 16.541;
    public static final double FIELD_WIDTH_METERS = 8.069;
    /** Centro del hub de la alianza AZUL, en el marco de campo (origen en esquina azul). */
    public static final Translation2d HUB_BLUE = new Translation2d(4.625, 4.035);
    /** Posicion de la salida del shooter respecto al centro del robot (x adelante, y izquierda). */
    public static final Translation2d ROBOT_TO_SHOOTER = new Translation2d(0.0, 0.0);

    // ---------------------------------- TABLAS DE TIRO ----------------------------------
    // distancia al hub (m) -> angulo de la capucha (grados) / RPM del shooter.
    // PLACEHOLDERS PARA QUE COMPILE Y NO TRUENE: reemplazar con puntos medidos en el robot.
    public static final InterpolatingDoubleTreeMap DUMPER_ANGLE_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap SHOOTER_RPM_MAP = new InterpolatingDoubleTreeMap();

    static {
        DUMPER_ANGLE_MAP.put(1.5, 10.0);
        DUMPER_ANGLE_MAP.put(3.0, 20.0);
        DUMPER_ANGLE_MAP.put(4.5, 30.0);

        SHOOTER_RPM_MAP.put(1.5, 2800.0);
        SHOOTER_RPM_MAP.put(3.0, 3300.0);
        SHOOTER_RPM_MAP.put(4.5, 3900.0);
    }


    /* 
    public static final InterpolatingDoubleTreeMap INTERPOLATION_MAP =
      new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap RPM_MAP = new InterpolatingDoubleTreeMap();

    static {
        RPM_MAP.put(1.81598, -2950.0);
        RPM_MAP.put(2.29161, -3390.0);
        RPM_MAP.put(2.57937, -3550.0);
        RPM_MAP.put(2.962915, -3750.0);
        RPM_MAP.put(3.466091, -4270.0);
        RPM_MAP.put(4.040353, -4450.0);
        RPM_MAP.put(4.547862, -4900.0);
        RPM_MAP.put(4.902743, -5000.0);
        RPM_MAP.put(5.340625, -5300.0);
    }

    static {
        INTERPOLATION_MAP.put(1.81598, -16.0);
        INTERPOLATION_MAP.put(2.29161, -17.5);
        INTERPOLATION_MAP.put(2.57937, -20.0);
        INTERPOLATION_MAP.put(2.962915, -22.0);
        INTERPOLATION_MAP.put(3.466091, -24.0);
        INTERPOLATION_MAP.put(4.040353, -26.85);
        INTERPOLATION_MAP.put(4.547862, -28.5);
        INTERPOLATION_MAP.put(4.902743, -31.7);
        INTERPOLATION_MAP.put(5.340625, -32.5);
    }

    */
    
}   