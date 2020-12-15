package aim4.im.intersectionarch;

import aim4.config.Constants;
import aim4.config.SimConfig;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class ArchRoadSpec {

    private final double speed;
    private final int inLanesCount;
    private final int outLanesCount;
    private final Constants.Direction direction;
    private final ArrayList<ArchLaneSpec> laneSpecs;
    private final int straightLaneOffset;
    private final double aheadReservationTime;
    private final Set<Integer> relativeIndicesOfWithTrafficTurnAllowedOnRedLanes;

    public ArchRoadSpec(Constants.Direction direction, double speed, int inLanesCount, int outLanesCount, ArrayList<ArchLaneSpec> laneSpecs, double aheadReservationTime, Set<Integer> relativeIndicesOfWithTrafficTurnAllowedOnRedLanes) {
        this.direction = direction;
        this.speed = speed;
        this.inLanesCount = inLanesCount;
        this.outLanesCount = outLanesCount;
        this.laneSpecs = laneSpecs;
        this.aheadReservationTime = aheadReservationTime;
        this.relativeIndicesOfWithTrafficTurnAllowedOnRedLanes = (relativeIndicesOfWithTrafficTurnAllowedOnRedLanes == null ? new HashSet<Integer>() : relativeIndicesOfWithTrafficTurnAllowedOnRedLanes);
        
        //calculate what the offset is in relative indices for lanes to go straight (this is needed later for mapping, as a car can't change lanes when going straight through an intersection)
        Integer tempOffset = null;
        for (ArchLaneSpec spec : laneSpecs) {
            EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>> byVehicles = spec.getTurnPoliciesAndRestrictions().get(direction);
            if (byVehicles != null) {
                for (Map<Integer, Integer> laneMapping : byVehicles.values()) {
                    for (Integer from : laneMapping.keySet()) {
                        if (tempOffset == null) {
                            tempOffset = from - laneMapping.get(from);
                        } else {
                            if (from - laneMapping.get(from) != tempOffset) {
                                throw new RuntimeException("Invalid mapping of lane with straight action found: " + direction.name() + " (" + from + ", " + laneMapping.get(from) + ") All straight lanes must share the same relative index offset. Expected offset was: " + tempOffset);
                            }
                        }
                    }
                }
            }
        }
        if (tempOffset == null) {
            straightLaneOffset = 0;
        } else {
            straightLaneOffset = tempOffset;
        }
    }

    public Constants.Direction getDirection() {
        return direction;
    }

    public double getSpeed() {
        return speed;
    }

    public int getInLanesCount() {
        return inLanesCount;
    }

    public int getOutLanesCount() {
        return outLanesCount;
    }

    public List<ArchLaneSpec> getlaneSpecs() {
        return Collections.unmodifiableList(laneSpecs);
    }

    public int getStraightLaneOffset() {
        return straightLaneOffset;
    }
    
    public double getAheadReservationTime() {
        return aheadReservationTime;
    }
    
    public Set<Integer> getSetOfLanesPotentiallySupportingWithTrafficTurnsOnRed() {
        return Collections.unmodifiableSet(relativeIndicesOfWithTrafficTurnAllowedOnRedLanes);
    }
}
