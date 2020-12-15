package aim4.config.ringbarrier;

import aim4.config.Constants;
import aim4.map.Road;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Iterator;
import java.util.PriorityQueue;
import java.util.Set;

/**
 *
 * @author Aaron Parks-Young
 */
public class ActuationTracker {

    private HashMap<Road, EnumMap<Constants.TurnDirection, PriorityQueue<ActuationExitPair>>> actuationsAndExits;
    private double currentTime = 0;
    private final double timeToClearOld = 0;

    public ActuationTracker() {
        actuationsAndExits = new HashMap<Road, EnumMap<Constants.TurnDirection, PriorityQueue<ActuationExitPair>>>();
    }

    public void logActuation(double currentSimTime, double actuationTime, double exitTime, Road rd, Set<Constants.TurnDirection> tds) {
        double diff = currentSimTime - currentTime;
        currentTime = currentSimTime;
        Set<Constants.TurnDirection> realTds = (tds.equals(Collections.EMPTY_SET) ? Constants.ACTABLE_TURN_DIRECTIONS : tds);

        if (actuationTime >= currentTime) {
            //check if inner map exists for the actuation, if not, create it.
            EnumMap<Constants.TurnDirection, PriorityQueue<ActuationExitPair>> innerMap = actuationsAndExits.get(rd);
            if (innerMap == null) {
                innerMap = new EnumMap<Constants.TurnDirection, PriorityQueue<ActuationExitPair>>(Constants.TurnDirection.class);
                actuationsAndExits.put(rd, innerMap);
            }

            if (realTds != null) {
                for (Constants.TurnDirection td : realTds) {
                    //check if list exists for the road and turn direction, if not, create it.
                    PriorityQueue<ActuationExitPair> queue = innerMap.get(td);
                    if (queue == null) {
                        innerMap.put(td, new PriorityQueue<ActuationExitPair>());
                        queue = innerMap.get(td);
                    } else {
                        pruneList(rd, td);
                    }

                    queue.add(new ActuationExitPair(actuationTime, exitTime));
                }
            }
        }
    }

    //removes leading elements in the list that occur before currentTime
    public void pruneList(Road rd, Constants.TurnDirection td) {
        EnumMap<Constants.TurnDirection, PriorityQueue<ActuationExitPair>> innerMap = actuationsAndExits.get(rd);
        if (innerMap == null) {
            return;
        }

        PriorityQueue<ActuationExitPair> queue = innerMap.get(td);
        if (queue == null || queue.isEmpty()) {
            return;
        }

        while (!queue.isEmpty()) {
            if (queue.peek().getActuationTime() + timeToClearOld < currentTime && queue.peek().getExitTimeForActuation() + timeToClearOld < currentTime) {
                queue.poll();
            } else {
                return;
            }
        }
    }

    /**
     * Lookup of actuations that occurred for a specific road for specific
     * turning directions for a specific timeframe.
     *
     * @param startTime begin time inclusive
     * @param endTime end time inclusive
     * @param rd
     * @param tds
     * @return
     */
    public PriorityQueue<ActuationExitPair> getActuations(double startTime, double endTime, Road rd, Set<Constants.TurnDirection> tds) {
        return getActuationsOrExits(startTime, endTime, rd, tds, true);
    }

    /**
     * Lookup of exits that occurred for a specific road for specific turning
     * directions for a specific timeframe.
     *
     * @param startTime begin time inclusive
     * @param endTime end time inclusive
     * @param rd
     * @param tds
     * @return
     */
    public PriorityQueue<ActuationExitPair> getExits(double startTime, double endTime, Road rd, Set<Constants.TurnDirection> tds) {
        return getActuationsOrExits(startTime, endTime, rd, tds, false);
    }

    //todo: may need to optimize this, the merging of queues isn't time/operation effecient
    /**
     * Lookup of actuations or exits that occurred for a specific road for
     * specific turning directions for a specific timeframe.
     *
     * startTime start begin time inclusive
     *
     * @param endTime end time inclusive
     * @param rd
     * @param tds
     * @param keyIfTrue compare keys of pairs if true (actuation times),
     * otherwise compare values (exit times)
     * @return
     */
    private PriorityQueue<ActuationExitPair> getActuationsOrExits(double startTime, double endTime, Road rd, Set<Constants.TurnDirection> tds, boolean keyIfTrue) {
        double actualStart = Math.max(startTime, currentTime);
        PriorityQueue<ActuationExitPair> returnQueue = new PriorityQueue<ActuationExitPair>();

        EnumMap<Constants.TurnDirection, PriorityQueue<ActuationExitPair>> innerMap = actuationsAndExits.get(rd);
        if (innerMap == null) {
            return returnQueue; //return empty list
        }

        for (Constants.TurnDirection td : tds) {
            PriorityQueue<ActuationExitPair> queue = innerMap.get(td);
            if (queue == null || queue.isEmpty()) {
                continue; //hop over this iteration of the loop
            }

            Iterator<ActuationExitPair> it = queue.iterator();
            while (it.hasNext()) {
                ActuationExitPair item = it.next();
                double timeToCompare = (keyIfTrue ? item.getActuationTime() : item.getExitTimeForActuation());
                if (timeToCompare > endTime) {
                    break;
                } else if (timeToCompare >= actualStart) {
                    returnQueue.add(item);
                }
            }
        }

        return returnQueue;
    }
}
