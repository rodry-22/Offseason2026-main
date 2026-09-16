package frc.robot.modules.superstructure.modules.IndexerModule;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.features.unitprocessor.Unit;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;

@Fallback
public interface IndexerIO extends IO<IndexerIO.IndexerInputs> {

    public static class IndexerInputs extends Data<IndexerInputs> {
        @Unit(value = "Volts", group = "Indexer")
        public double rollerVolts = 0;

        @Unit(value = "RPS", group = "Indexer")
        public double rollerRPS = 0;

        @Unit(value = "Volts", group = "Indexer")
        public double indexVolts = 0;

        @Unit(value = "RPS", group = "Indexer")
        public double indexRPS = 0;


        @Override
        public IndexerInputs snapshot() {
            IndexerInputs clone = new IndexerInputs();
            clone.rollerVolts = this.rollerVolts;
            clone.rollerRPS = this.rollerRPS;
            clone.indexVolts = this.indexVolts;
            clone.indexRPS = this.indexRPS;
            return clone;
        }
    }

    public void applyRollers(@Unit(value = "Volts", group = "Indexer") double volts);
    /**
     * Sets the voltage of the rollers motor.
     * @param volts The voltage to apply to the rollers motor.
    */

    public void applyIndex(@Unit(value = "Volts", group = "Indexer") double volts);
    /**
     * Sets the voltage of the index motor.
     * @param volts The voltage to apply to the indexer motor.
     */

    public void setRollers(@Unit(value = "RPS", group = "Indexer") double RPS);
    /**
     * Sets the rollers to a given RPS with a closed loop controller.
     * @param RPS The desired RPS.
     */

    public void setIndex(@Unit(value = "RPS", group = "Indexer") double RPS);
    /**
     * Sets the index to a given RPS with a closed loop controller.
     * @param RPS The desired RPS.
     */

    public void stopRollers();
    /**
     * Stops the rollers motor.
     */
    public void stopIndex();
    /**
     * Stops the index motor.
     */

    public void stopAll();
    /**
     * Stops the the indexer system (both roller and indexer)
     */
}
