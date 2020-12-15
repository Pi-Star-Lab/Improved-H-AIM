package aim4.config.ringbarrier;

import aim4.config.Constants;
import static aim4.config.Constants.ACTABLE_TURN_DIRECTIONS;
import aim4.config.TrafficSignal;
import aim4.map.Road;
import aim4.map.lane.Lane;
import java.util.Collections;
import java.util.Set;

/**
 *
 * @author Aaron Parks-Young
 */
public class RBPhaseBarrier extends RBPhaseSegment {

    public RBPhaseBarrier(double time, TrafficSignal color) {
        super(null, time, color, true, true, true, false, false);
    }

    public RBPhaseBarrier(RBPhaseSegment phaseToCopy) {
        super(phaseToCopy);
    }

    @Override
    public Road getRoad() {
        return (previousPhaseSegment == null ? null : previousPhaseSegment.getRoad());
    }

    @Override
    public boolean isBarrier() {
        return true;
    }

    @Override
    public Set<Constants.TurnDirection> getTurnDirectionsForPhaseSegment() {
        return Collections.unmodifiableSet(ACTABLE_TURN_DIRECTIONS); //all directions are affected for a barrier
    }

    public Set<Lane> getLanesInPhaseSegment() {
        throw new UnsupportedOperationException("Barriers cannot return the lanes in their segment as they are coded to affect all lanes. Try using the preceding green phase segment to get affiliated lanes.");
    }
}
