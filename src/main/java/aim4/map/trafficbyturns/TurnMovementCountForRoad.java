package aim4.map.trafficbyturns;

import aim4.config.Constants;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class TurnMovementCountForRoad implements Comparable<Double> {

    private final double timeoffset;
    private int total;
    private final Map<Constants.TurnDirection, Integer> acttoval;
    private boolean retotal;
    private double timeinterval;
    public static final double DEFAULT_INTERVAL = Double.POSITIVE_INFINITY;

    //represents all turning actions for a road in a given time slot.
    public TurnMovementCountForRoad(double timeoffset) {
        if (timeoffset < 0) {
            throw new IllegalArgumentException("Time offset cannot be less than 0. Provided time offset: " + timeoffset);
        } else {
            this.timeoffset = timeoffset;
        }
        total = 0;
        acttoval = new EnumMap<Constants.TurnDirection, Integer>(Constants.TurnDirection.class);
        retotal = false;
        timeinterval = DEFAULT_INTERVAL;
    }

    public double getTimeInterval() {
        return timeinterval;
    }

    public void setTimeInterval(double timeamt) {
        if (timeamt < 0) {
            throw new IllegalArgumentException("Time imterval cannot be less than 0. Provided time interval: " + timeamt);
        }
        this.timeinterval = timeamt;
    }

    /**
     *
     * @return amount of time this time slot spans
     */
    public double getEndTime() {
        return timeoffset + timeinterval;
    }

    /**
     *
     * @return time at which this time slot starts
     */
    public double getTimeOffset() {
        return timeoffset;
    }

    //should return false if key was already added
    public boolean putActValIfAbsent(Constants.TurnDirection act, int val) {
        boolean retval = acttoval.putIfAbsent(act, val) == null;
        if (retval) {
            retotal = true;
        }
        return retval;
    }

    public void putActVal(Constants.TurnDirection act, int val) {
        acttoval.put(act, val);
        retotal = true;
    }

    public int getTotal() {
        if (retotal) {
            total = 0;
            ArrayList<Constants.TurnDirection> keys = new ArrayList<Constants.TurnDirection>(acttoval.keySet());
            for (int i = 0; i < keys.size(); i++) {
                total += acttoval.get(keys.get(i));
            }
            retotal = false;
        }

        return total;
    }

    public int getFromAction(Constants.TurnDirection act) {
        return acttoval.get(act);
    }

    public Set<Constants.TurnDirection> getActions() {
        return new HashSet<Constants.TurnDirection>(acttoval.keySet());
    }

    @Override
    public int compareTo(Double time) {
        return Double.compare(timeoffset, time);
    }
}
