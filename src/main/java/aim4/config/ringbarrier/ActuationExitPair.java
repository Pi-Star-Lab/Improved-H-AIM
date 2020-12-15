package aim4.config.ringbarrier;

import aim4.util.LimitedPairImplementation;

public class ActuationExitPair extends LimitedPairImplementation<Double, Double> implements Comparable<ActuationExitPair> {

    public ActuationExitPair(Double key, Double value) {
        super(key, value);
    }

    public double getActuationTime() {
        return getKey();
    }

    /**
     * 
     * @return the time at which the vehicle triggering the actuation will clear the intersection 
     */
    public double getExitTimeForActuation() {
        return getValue();
    }

    @Override
    public int compareTo(ActuationExitPair t) {
        return getKey().compareTo(t.getKey());
    }
}
