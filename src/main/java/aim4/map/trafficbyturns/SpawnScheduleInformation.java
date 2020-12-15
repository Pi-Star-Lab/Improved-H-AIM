package aim4.map.trafficbyturns;

import aim4.config.Constants;
import aim4.util.Util;

public class SpawnScheduleInformation implements Comparable<SpawnScheduleInformation>{
    /**
     * records the time a vehicle should spawn
     */
    private final Double spawnTime;
    /**
     * records the direction a vehicle should spawn from
     */
    private final Constants.TurnDirection td;

    /**
     * Gets the time the vehicle in this pairing should be spawned
     * @return the time a vehicle should be spawned
     */
    public double getSpawnTime() {
        return spawnTime;
    }

    /**
     * Gets the direction a vehicle should be turning when arriving at the *single* (only 1 intersection is currently supported) intersection
     * @return the turn direction a vehicle spawning at spawnTime should turn.
     */
    public Constants.TurnDirection getTD() {
        return td;
    }

    /**
     * 
     * @param spawnTime the time a vehicle should be spawned.
     * @param td the turn direction a vehicle spawning at spawnTime should turn.
     */
    public SpawnScheduleInformation(double spawnTime, Constants.TurnDirection td) {
        this.spawnTime = spawnTime;
        //picks an action if needed from a compound action.
        this.td = Util.selectRandomFromCompoundAction(td);
    }

    @Override
    public int compareTo(SpawnScheduleInformation t) {
        return Double.compare(spawnTime, t.spawnTime);
    }
}
