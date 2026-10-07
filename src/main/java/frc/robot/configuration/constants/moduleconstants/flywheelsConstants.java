package frc.robot.configuration.constants.moduleconstants;

import com.ctre.phoenix6.signals.InvertedValue;

public class flywheelsConstants {
    

    public class shooterWheelsConstants{

        public static final int shooterLeaderID = 19;
        public static final int shooterFollowerID = 20;

        public static final double SupplyCurrentLimit = 0;
        public static final boolean SupplyCurrentLimitEnable = true;

        public static final double StatorCurrentLimit = 0;
        public static final boolean StatorCurrentLimitEnable = true;

        public static final double kRPMTolerance = 50;

        // ---------------- SHOOT ----------------
        /** RPM fijo del disparo manual (trigger izquierdo). */
        public static final double kShootRPM = 6000;
        /** Tiempo (s) que las RPM deben estar dentro de tolerancia antes de arrancar los rollers. */
        public static final double kReadyDebounceSec = 0.15;
        /** PRUEBA de alcance: duty cycle maximo de las flywheels (1.0 = 100%, sin PID). */
        public static final double kMaxDuty = 1.0;

        // FF + PID del Talon (Slot0, VelocityVoltage). Unidades de Phoenix: volts y RPS del rotor.
        // kS = 0.4 y kP ~ 0.5 salen de las flywheels 2024 (mismo Kraken 1:1: kS 0.4, kV 0.12, kP 0.6).
        // kV = 0.1277 V/RPS es el medido de tu robot. FF a 5000 RPM = 0.4 + (5000/60)*0.1277 = 11.0 V.
        public static final double kS = 0.2;
        public static final double kV = 0.65;
        public static final double kP = 0.85;
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

        public static InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
        public static double StatorCurrentLimit = 40;
        public static double SupplyCurrentLimit = 60;
  }

        

    }

}