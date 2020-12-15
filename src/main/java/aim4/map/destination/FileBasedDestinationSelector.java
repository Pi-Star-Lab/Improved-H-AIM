package aim4.map.destination;

import aim4.config.Constants;
import aim4.im.IntersectionManager;
import aim4.map.Road;
import aim4.map.lane.Lane;
import aim4.util.Registry;
import aim4.util.Util;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Random;

public class FileBasedDestinationSelector implements DestinationSelector {
    /////////////////////////////////
    // PRIVATE FIELDS
    /////////////////////////////////

    /**
     * Map of roads to a map of actions from that road to the destination road,
     * assumes a single intersection in a grid(ish) format
     */
    private final Map<Road, Map<Constants.TurnDirection, Road>> destinations;

    /////////////////////////////////
    // CLASS CONSTRUCTORS
    /////////////////////////////////
    /**
     * Create a new file based destination selector from the given Layout.
     *
     * @param destinations Map of maps. Uses current road and action to
     * determine destination road.
     */
    public FileBasedDestinationSelector(Map<Road, Map<Constants.TurnDirection, Road>> destinations) {
        this.destinations = destinations;
    }

    /**
     * Create a new file based destination selector from the given layout using
     * the layout IntersectionManager Registry. Currently only allows 1
     * intersection manager to have been specified in the registry.
     *
     * @param intersections IntersectionManager Registry used to create mapping
     * for selection of destination roads by turn action from incoming road to
     * an intersection. Must not be <code>null</code>.
     */
    public FileBasedDestinationSelector(Registry<IntersectionManager> intersections) {
        List<IntersectionManager> ims = intersections.getValues();
        if (ims == null) {
            throw new IllegalArgumentException("Null registry of intersections provided to FileBasedDestinationSelector. The registry must not be null.");
        } else if (ims.isEmpty()) {
            throw new IllegalArgumentException("Empty registry of intersections provided to FileBasedDestinationSelector. The registry must not be empty.");
        } else if (ims.size() > 1) {
            throw new IllegalArgumentException("FileBasedDestinationSelector only currently supports a single intersection manager, multiple intersection managers have been specified.");
        }
        destinations = new HashMap<Road, Map<Constants.TurnDirection, Road>>();
        //put in a for loop for future expanadability
        for (IntersectionManager im : ims) {
            for (Road inRoad : im.getIntersection().getEntryRoads()) {
                EnumMap<Constants.TurnDirection, Road> innerMap = new EnumMap<Constants.TurnDirection, Road>(Constants.TurnDirection.class);
                for (Road outRoad : im.getIntersection().getExitRoads()) {
                    //todo uturns are currently not allowed, logic short circuits such that if road doesn't have a dual it's unlikely a u-turn will be selected
                    if (inRoad.getDual() == null || inRoad.getDual() != outRoad) {
                        if (inRoad == outRoad) {
                            innerMap.put(Constants.TurnDirection.STRAIGHT, inRoad);
                        } else  {
                            Constants.TurnDirection td = im.getIntersection().calcTurnDirection(inRoad.getIndexLane(), outRoad.getIndexLane());
                            if (innerMap.containsKey(td)) {
                                throw new RuntimeException("Multiple roads found that have the same turn direction from a given road in FileBasedDestinationSelector constructor. This is not allowed. Incoming road: " + inRoad.getName() + " Outgoing road 1: " + innerMap.get(td).getName() + " Outgoing road 2: " + outRoad.getName());
                            } else if (td == Constants.TurnDirection.STRAIGHT_ALL || td == Constants.TurnDirection.STRAIGHT_RIGHT || td == Constants.TurnDirection.STRAIGHT_LEFT || td == Constants.TurnDirection.U_TURN) {
                                throw new RuntimeException("im.getIntersection().calcTurnDirection() returned a compound or disallowed action: " + td.name() + " from road " + inRoad.getName() + " to " + outRoad.getName());
                            } else {
                                innerMap.put(td, outRoad);
                            }
                        }
                    }
                    destinations.put(inRoad, innerMap);
                }
            }
        }
    }

    /////////////////////////////////
    // PUBLIC METHODS
    /////////////////////////////////
    /**
     * {@inheritDoc}
     */
    @Override
    public Road selectDestination(Lane currentLane) {
        Road currentRoad = currentLane.getContainingRoad();
        List<Constants.TurnDirection> acts = Arrays.asList(Constants.TurnDirection.values());
        Collections.shuffle(acts, Util.RANDOM_NUM_GEN);
        Iterator<Constants.TurnDirection> actiter = acts.iterator();
        while (actiter.hasNext()) {
            Constants.TurnDirection act = Util.selectRandomFromCompoundAction(actiter.next());
            Map<Constants.TurnDirection, Road> actiontodest = destinations.get(currentRoad);
            if (actiontodest != null) {
                Road dest = actiontodest.get(act);
                if (dest != null) {
                    return dest;
                }
            }
        }
        return null;
    }

    /**
     * Select the Road which the given Vehicle should use as its destination.
     *
     * @param currentRoad the road the Vehicle is currently on
     * @param act the action the vehicle is supposed to take (as with UDOT turn
     * volume files)
     * @return the Road which the Vehicle should use as its destination
     */
    public Road selectDestination(Road currentRoad, Constants.TurnDirection act) {
        return destinations.get(currentRoad).get(Util.selectRandomFromCompoundAction(act));
    }

    @Override
    public List<Road> getPossibleDestination(Lane currentLane) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
}
