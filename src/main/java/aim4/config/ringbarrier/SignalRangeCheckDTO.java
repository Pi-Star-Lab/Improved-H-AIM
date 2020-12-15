package aim4.config.ringbarrier;

public class SignalRangeCheckDTO {

    public double time;
    public HistoricalRBSegmentInformation segmentInfo;
    public RBPhaseSegment segment;
    public int index;
    public RBRing ring;

    public SignalRangeCheckDTO(double time, HistoricalRBSegmentInformation segmentInfo, RBPhaseSegment segment, int index, RBRing ring) {
        this.time = time;
        this.segmentInfo = segmentInfo;
        this.segment = segment;
        this.index = index;
        this.ring = ring;
    }

    public SignalRangeCheckDTO(SignalRangeCheckDTO other) {
        time = other.time;
        segmentInfo = other.segmentInfo;
        segment = other.segment;
        index = other.index;
        ring = other.ring;
    }
}
