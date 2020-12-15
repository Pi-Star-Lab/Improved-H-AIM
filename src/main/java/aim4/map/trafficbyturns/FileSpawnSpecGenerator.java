package aim4.map.trafficbyturns;

import aim4.config.SimConfig;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpecGenerator;

public abstract class FileSpawnSpecGenerator implements SpawnSpecGenerator {

    /**
     * Tracked Current Simtime
     */
    protected double trtime;

    /**
     * Gets the total number of vehicles that have been designated (as in,
     * returned by this SpawnSpecGen)to have been spawned by this SpawnSpecGen.
     *
     * @return the number of vehicles that have been spawned
     */
    public abstract int getTotalVehiclesSpawned();

    /**
     * Gets the total number of vehicles that have been scheduled to be spawned
     * by this SpawnSpecGen.
     *
     * @return the number of vehicles that have been scheduled spawned
     */
    public abstract int getTotalVehiclesScheduled();

    /**
     * Gets the number of vehicles that remain to be spawned in this time
     * slot by this SpawnSpecGen.
     *
     * @return the number of vehicles that are left to be spawned in this time
     * slot
     */
    public abstract int getVehiclesLeftInCurrentTimeSlot();

    @Override
    public SpawnPoint.SpawnSpec act(SpawnPoint spawnPoint, double timeStep, SimConfig.VEHICLE_TYPE vehicleType, Road destinationRoad) {
        throw new UnsupportedOperationException("Specifying a destination road outside of turn movements file is not permitted with a FileSpawnSpecGenerator.");
    }

}
