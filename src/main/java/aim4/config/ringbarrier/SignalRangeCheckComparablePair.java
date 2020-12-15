package aim4.config.ringbarrier;

import aim4.util.LimitedPairImplementation;

public class SignalRangeCheckComparablePair extends LimitedPairImplementation<Double, RBRing> implements Comparable<SignalRangeCheckComparablePair>{

    public SignalRangeCheckComparablePair(Double key, RBRing value) {
        super(key, value);
        if (key == null || value == null) {
            throw new IllegalArgumentException("Neither the key nor the value may be null.");
        }
    }

    @Override
    public int compareTo(SignalRangeCheckComparablePair t) {
        return getKey().compareTo(t.getKey());
    }

}
