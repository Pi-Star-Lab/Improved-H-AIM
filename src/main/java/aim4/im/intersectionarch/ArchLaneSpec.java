package aim4.im.intersectionarch;

import aim4.config.Constants;
import aim4.config.SimConfig;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

public class ArchLaneSpec {

    /**
     * Maps outgoing direction (which road at the intersection) of lane to lane-to-lane mapping through vehicle type
     */
    private final EnumMap<Constants.Direction, EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>>> turnPoliciesAndRestrictions;

    public ArchLaneSpec() {
        this.turnPoliciesAndRestrictions = new EnumMap<Constants.Direction, EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>>>(Constants.Direction.class);
    }

    public void addPolicyAndRestriction(Constants.Direction exitDirection, SimConfig.VEHICLE_TYPE vType, AllowedTurn turnPolicyTuple) {
        EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>> policyAndRestrictions;
        if (turnPoliciesAndRestrictions.containsKey(exitDirection)) {
            policyAndRestrictions = turnPoliciesAndRestrictions.get(exitDirection);
        } else {
            policyAndRestrictions = new EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>>(SimConfig.VEHICLE_TYPE.class);
            turnPoliciesAndRestrictions.put(exitDirection, policyAndRestrictions);
        }

        if (!policyAndRestrictions.containsKey(vType)) {
            HashMap<Integer, Integer> laneMapping = new HashMap<Integer, Integer>();
                if (!laneMapping.containsKey(turnPolicyTuple.getFrom())) {
                    laneMapping.put(turnPolicyTuple.getFrom(), turnPolicyTuple.getTo());
                } else {
                    throw new RuntimeException("1-to-N mapping of lane turns/restrictions found: " + exitDirection + " " + vType + ". The following conflicts as only 1-to-1 mappings are allowed. Previously recorded: (" + turnPolicyTuple.getFrom() + ", " + laneMapping.get(turnPolicyTuple.getFrom()) + ") + Encountered: (" + turnPolicyTuple.getFrom() + ", " + turnPolicyTuple.getTo() + ").");
                }
            
            policyAndRestrictions.put(vType, laneMapping);
        } else {
            throw new RuntimeException("Duplicate policy/restriction for a vehicle type found when specifying turning policy/lane restrictions: " + exitDirection.name() + " " + vType.name() + ". This is currently not supported.");
        }
    }
    
    public Map<Constants.Direction, EnumMap<SimConfig.VEHICLE_TYPE, Map<Integer, Integer>>> getTurnPoliciesAndRestrictions() {
        return Collections.unmodifiableMap(turnPoliciesAndRestrictions);
    }
}
