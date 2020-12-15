package aim4.im.intersectionarch;

public class AllowedTurn {

    /**
     * Lane index (with 0 being leftmost, 1 being one right of that, etc.) from
     * which this turn/action is allowed.
     */
    private final int from;
    /**
     * Lane index (with 0 being leftmost, 1 being one right of that, etc.) on
     * some road (mapped by other objects) to which this turn/action is allowed.
     */
    private final int to;

    public AllowedTurn(int from, int to) {
        if (from < 0 || to < 0) {
            throw new IllegalArgumentException("Both from and to lane indices in allowed turn must be >= 0. Provided from: " + from + "Provided to: " + to);
        } else {
            this.from = from;
            this.to = to;
        }
    }

    public int getFrom() {
        return from;
    }

    public int getTo() {
        return to;
    }

    public String getStringRepresentation() {
        return "(" + from + ", " + to + ")";
    }

}
