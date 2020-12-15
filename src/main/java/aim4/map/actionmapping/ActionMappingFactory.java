package aim4.map.actionmapping;

import aim4.config.Constants;
import java.util.HashMap;
import java.util.Map;

public class ActionMappingFactory {
    //todo, figure out how total can be mapped instead of being a reserved word...
    public static Map<String, Constants.TurnDirection> getUDOTActionMapping() {
        Map<String, Constants.TurnDirection> mp = new HashMap<String, Constants.TurnDirection>();
        mp.put("T", Constants.TurnDirection.STRAIGHT);
        mp.put("TL", Constants.TurnDirection.STRAIGHT_LEFT);
        mp.put("TR", Constants.TurnDirection.STRAIGHT_RIGHT);
        mp.put("L", Constants.TurnDirection.LEFT);
        mp.put("R", Constants.TurnDirection.RIGHT);
        mp.put("TLR", Constants.TurnDirection.STRAIGHT_ALL);
        return mp;
    }

    //todo, figure out how total can be mapped instead of being a reserved word...
    public static Map<String, Constants.TurnDirection> getDefaultActionMapping() {
        Map<String, Constants.TurnDirection> mp = new HashMap<String, Constants.TurnDirection>();
        for (Constants.TurnDirection act : Constants.TurnDirection.values()) {
            mp.put(act.name(), act);
        }
        return mp;
    }
}
