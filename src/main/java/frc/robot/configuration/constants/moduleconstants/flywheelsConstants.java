package frc.robot.configuration.constants.moduleconstants;

import com.ctre.phoenix6.signals.InvertedValue;

public class flywheelsConstants {
    

    public class shooterWheelsConstants{

        public static final int shooterLeaderID = 19;
        public static final int shooterFollowerID = 20;

        // OJO: un limite de 0 A con Enable = true deja el motor SIN corriente (no gira / sin fuerza).
        // Para "sin limite" no pongas 0: pon un valor alto o Enable = false. Valores de MARS-base:
        public static final double SupplyCurrentLimit = 120;
        public static final boolean SupplyCurrentLimitEnable = true;

        public static final double StatorCurrentLimit = 90;
        public static final boolean StatorCurrentLimitEnable = true;

        /**
         * Sentido de giro del shooter. En MARS-base el disparo real es en sentido NEGATIVO
         * (RPM_MAP de -2950 a -5300, ShooterRMPtest a -3000 RPM, y clearFuel usa +12 V para sacar).
         * Con Clockwise_Positive el sentido de disparo queda en POSITIVO y el resto del codigo
         * (RPM positivos, tablas, duty +1.0) no cambia. Si al probar gira al reves, ponlo en
         * CounterClockwise_Positive.
         */
        public static final InvertedValue kShooterInverted = InvertedValue.Clockwise_Positive;

        public static final double kRPMTolerance = 50;

        // ---------------- SHOOT ----------------
        /** RPM fijo del disparo manual (trigger izquierdo). 6000 RPM no se alcanza: pide 12.8 V solo de FF. */
        public static final double kShootRPM = 6000;
        /**
         * Relacion de engranes del shooter = vueltas del MOTOR por cada vuelta de la RUEDA.
         * > 1 = reduccion (la rueda gira MAS LENTO que el motor), < 1 = aumento, 1 = directo.
         * PONER LA REAL. Con esto TODOS los RPM del codigo (setpoints, tablas, telemetria
         * velocityRPM) pasan a ser RPM de la RUEDA; el RPM del motor sale en motorRPM.
         */
        public static final double kGearRatio = 1.0; // reduccion 3:1 (medido: la rueda llega a ~1600 RPM)

        /**
         * Fraccion de la velocidad maxima teorica (a 12 V) que se permite pedir. Con carga el motor
         * no llega a la teorica (medido con 3:1: ~1600 de 1849 RPM de rueda = 87%), asi que 0.9 pedia
         * un RPM que NUNCA se alcanzaba y los rollers no arrancaban. 0.8 deja margen para el PID.
         */
        public static final double kMaxSpeedMargin =  0.9;

        /** Tiempo (s) que las RPM deben estar dentro de tolerancia antes de arrancar los rollers. */
        public static final double kReadyDebounceSec = 0.15;
        /** Maximo tiempo (s) que shoot espera a que la rueda llegue al RPM; despues alimenta de todos modos. */
        public static final double kSpinUpTimeoutSec = 3.0;
        /** PRUEBA de alcance: duty cycle maximo de las flywheels (1.0 = 100%, sin PID). */
        public static final double kMaxDuty = 1.0;

        // FF + PID del Talon (Slot0, VelocityVoltage). Unidades de Phoenix: volts y RPS del rotor.
        // kV = 0.1277 V/RPS es el de MARS-base (Kraken 1:1; 12 V / 100 RPS libres = 0.12 teorico).
        // OJO: kV = 0.65 pedia 54 V a 5000 RPM -> el motor se quedaba SIEMPRE a voltaje maximo y el PID
        // no regulaba. FF a 5000 RPM = 0.2 + (5000/60)*0.1277 = 10.8 V.
        public static final double kS = 0.2;
        public static final double kV = 0.12765427;

        /** RPM maximo de la RUEDA que el codigo permite pedir, calculado con kS, kV, 12 V y la relacion. */
        public static final double kMaxWheelRPM = ((12.0 - kS) / kV) * 60.0 / kGearRatio * kMaxSpeedMargin;
        public static final double kP = 0.95;
        public static final double kI = 0;

        public static final double kD = 0;

        public static double idleVoltage = 0;

        public static final double RPM_0_1 = 0;
        public static final double RPM_1_2 = 0;
        public static final double RPM_2_3 = 0;
        public static final double RPM_3_4 = 0;
        public static final double RPM_4_5 = 0;

        //sim
        public static final double kGearing = 4;
        public static final double kMOI = 0.002;
        //sim

        public class IntakeWheelsConstants {
        public static final int IntakeWheels_ID = 14;

        public static final double kIntakeRPMeat = -2000; // negativo
        public static final double kIntakeRPMvomiatr = 2000; // negativo
        public static final double kS = 0.1;
        public static final double kV = 0.12;          
        public static final double kP = 0.11;
        public static InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
        public static double StatorCurrentLimit = 40;
        public static double SupplyCurrentLimit = 60;
  }

        

    }

}