package aim4.util;

/**
 * HPRC version of Java didn't seem to have JavaFx libraries installed, so, needed a pair representation
 */
public class LimitedPairImplementation<T1, T2> {
    private final T1 first;
    private final T2 second;
    
    public LimitedPairImplementation(T1 first, T2 second) {
        this.first = first;
        this.second = second;
    }
    
    public T1 getKey() {
        return first;
    }
    
    public T2 getValue() {
        return second;
    }
}
