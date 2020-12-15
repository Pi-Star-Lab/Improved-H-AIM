package aim4.gui.frame.signalvisualization;

import aim4.config.Constants;
import aim4.config.ringbarrier.HistoricalRBSegmentInformation;
import aim4.config.ringbarrier.RingAndBarrier;
import aim4.gui.Viewer;
import aim4.util.LimitedPairImplementation;
import aim4.util.Util;
import java.awt.Color;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;

public class SignalVisualizationPanel extends javax.swing.JPanel {

    private static final Color TURN_DIRECTION_INACTIVE_COLOR = Color.RED;
    private static final Color RB_PHASE_BG_COLOR = Color.BLACK;
    private static final double DRAW_PHASE_HISTORY_MIN_HEIGHT_MOD_PERC = 0.17f;
    private static final double DRAW_MIN_HEIGHT_MOD_PERC = 0.10f;
    private static final double DRAW_HISTORY_RB_BARRIER_MOD_PERC = 0.650f;
    private static final double DRAW_MAX_HEIGHT_MOD_PERC = 0.90f;
    private static final double DRAW_MAX_WIDTH_PERC = 0.95f;
    private static final double RING_AND_BARRIER_MAX_DRAW_WIDTH_PERC = 0.8f;
    private static final double DRAW_MIN_WIDTH_PERC = 0.05f;
    private static final double TRIANGLE_POINT_DISTANCE_PERC = 0.1f;
    private static final double PERC_OF_CONTAINING_BOX_FOR_TURN_DIRECTIONS = 0.2f;

    private static final int MAJOR_RULER_TICKS_EVERY_SECONDS = 10;
    private static final int MINOR_RULER_TICKS_EVERY_SECONDS = 1;
    private static final Color RULER_COLOR = Color.BLACK;

    private final Object mutex = new Object(); //mutex to keep thread that updates the object on sim step from concurrently modifying/viewing the phaseSegmentDepictions list from conflicting with the paint thread of this panel 

    private ArrayList<LinkedList<ActuatedSignalVisualizationRepresentation>> phaseSegmentDepictions;
    private ArrayList<LinkedList<Set<Constants.TurnDirection>>> turnDirectionsByPhases;
    private ArrayList<LinkedList<Double>> entryHeadingsByPhases;
    private ArrayList<List<Set<Long>>> segmentIdsInPhases;
    private double secondsTimeWindowToSupport;
    private double maxRingHeight;
    private boolean clearAll;

    /**
     * The current viewer object
     */
    private Viewer viewer;
    /**
     * The ring and barrier
     */
    private RingAndBarrier rb;

    /**
     * Creates new form SignalVisualizationPanel
     *
     */
    public SignalVisualizationPanel() {
    }

    public void initialize(Viewer viewer, double secondsTimeWindowToSupport) {
        this.viewer = viewer;
        initComponents();

        this.secondsTimeWindowToSupport = Math.max(0f, secondsTimeWindowToSupport);

        maxRingHeight = 50f;
        phaseSegmentDepictions = null;
        turnDirectionsByPhases = null;
        setRingAndBarrier(null);
        clearAll = false;
    }

    public void setRingAndBarrier(RingAndBarrier rb) {
        synchronized (mutex) {
            if (this.rb != rb) {
                this.rb = rb;
                if (this.rb != null && this.rb.getNumberOfRings() > 0) {
                    phaseSegmentDepictions = new ArrayList<LinkedList<ActuatedSignalVisualizationRepresentation>>(this.rb.getNumberOfRings());
                    turnDirectionsByPhases = this.rb.getTurnDirectionsByRingSeparatedByPhaseSortedByRingId();
                    entryHeadingsByPhases = this.rb.getStreetDirectionsByRingSeparatedByPhaseSortedByRingId();
                    segmentIdsInPhases = this.rb.getSegmentIdsInPhases();
                } else if (this.rb != null && this.rb.getNumberOfRings() == 0) {
                    phaseSegmentDepictions = new ArrayList<LinkedList<ActuatedSignalVisualizationRepresentation>>();
                    turnDirectionsByPhases = new ArrayList<LinkedList<Set<Constants.TurnDirection>>>();
                    entryHeadingsByPhases = new ArrayList<LinkedList<Double>>();
                    segmentIdsInPhases = new ArrayList<List<Set<Long>>>();
                } else {
                    //flip these off so new information isn't continually fetched from the old ring and barrier.
                    phaseSegmentDepictions = null;
                    segmentIdsInPhases = null;
                }
                clearAll = true;
            }
        }
    }

    private double getSecondsPerPixelScale(double workingWidth, double secondsTimeWindowToSupport) {
        return secondsTimeWindowToSupport / workingWidth;
    }

    private double getWorkingHeight(double minHeightPerc, double maxHeightPerc) {
        double height = getSize().height;
        double realMin = (minHeightPerc <= maxHeightPerc ? minHeightPerc : maxHeightPerc);
        double realMax = (minHeightPerc <= maxHeightPerc ? maxHeightPerc : minHeightPerc);
        return (height * realMax) - (height * realMin);
    }

    private double getWorkingWidth(double minWidthPerc, double maxWidthPerc) {
        double width = getSize().width;
        double realMin = (minWidthPerc <= maxWidthPerc ? minWidthPerc : maxWidthPerc);
        double realMax = (minWidthPerc <= maxWidthPerc ? maxWidthPerc : minWidthPerc);
        return (width * realMax) - (width * realMin);
    }

    public void updateCurrentPhaseSegmentDepictions() {
        if (phaseSegmentDepictions != null && rb != null) {
            //check if this is the first time we're adding to the depictions or if we're updating
            synchronized (mutex) {
                ArrayList<LinkedList<HistoricalRBSegmentInformation>> phaseSegmentTimingInfo = rb.getCurrentSegmentTimingInfoSortedByRingId();
                ArrayList<LinkedList<HistoricalRBSegmentInformation>> previousPhaseSegmentTimingInfo = rb.getPreviousSegmentTimingInfoSortedByRingId();

                if (phaseSegmentDepictions.isEmpty()) {
                    clearAll = true;
                    int index = 0;
                    for (LinkedList<HistoricalRBSegmentInformation> listWithOnlyOneElement : phaseSegmentTimingInfo) {
                        LinkedList<ActuatedSignalVisualizationRepresentation> innerList = new LinkedList<ActuatedSignalVisualizationRepresentation>();
                        phaseSegmentDepictions.add(innerList);
                        handleHistoricalInfo(previousPhaseSegmentTimingInfo, index, innerList);
                        HistoricalRBSegmentInformation sti = listWithOnlyOneElement.get(0);
                        innerList.add(new ActuatedSignalVisualizationRepresentation(sti));
                        ++index;
                    }
                } else {
                    //create an iterator to step through all the representations of phase segments we're holding onto
                    ListIterator<LinkedList<ActuatedSignalVisualizationRepresentation>> lit = phaseSegmentDepictions.listIterator();
                    while (lit.hasNext()) {
                        int index = lit.nextIndex();
                        HistoricalRBSegmentInformation currentSegmentInfoForRing = phaseSegmentTimingInfo.get(index).get(0); //get the current element as far as the ring object is actually concerned
                        LinkedList<ActuatedSignalVisualizationRepresentation> currentList = lit.next();//look through the list of local phase segment representations for a particular ring
                        if (currentList.getLast().getIdOfSegmentInformation() == currentSegmentInfoForRing.getId()) { //update the most recent phase segment if it is the current segment
                            currentList.getLast().setNewPhaseSegmentInfo(currentSegmentInfoForRing);
                        } else { //it is not the current segment, so we need to add a new one and move on
                            handleHistoricalInfo(previousPhaseSegmentTimingInfo, index, currentList);
                            currentList.add(new ActuatedSignalVisualizationRepresentation(currentSegmentInfoForRing)); //add the new current element to the end of the list
                            removeOldSegmentDepictionsForRing(currentList);
                        }
                    }

                }
            }
        }
    }

    private void handleHistoricalInfo(ArrayList<LinkedList<HistoricalRBSegmentInformation>> previousPhaseSegmentTimingInfo, int index, LinkedList<ActuatedSignalVisualizationRepresentation> currentList) {
        LinkedList<HistoricalRBSegmentInformation> historicalSegmentDataForRing = previousPhaseSegmentTimingInfo.get(index); //this is the historical segment data for the ring at the current index
        ListIterator<HistoricalRBSegmentInformation> historicalLit = historicalSegmentDataForRing.listIterator(); //iterator to step through the historical segment data for the ring at the current index
        while (historicalLit.hasNext()) {
            /*if (currentList.getLast().getIdOfSegmentInformation() != previousSegmentInfo.getId()) {
                                        throw new RuntimeException("Phase segment likely missed in signal visualization ID. Previous segment info and previous segment info reported by ring do not have the same ID.");
                                    }*/
            int historicalIndex = historicalLit.nextIndex(); //cache at the index within the historical segment data for the ring at the current index
            HistoricalRBSegmentInformation previousSegmentInfo = historicalLit.next(); //get the next element
            //if the "current" phase segment is now historical, transition it by updating it one last time, then add one to be the most current
            if (historicalIndex == 0 && currentList.size() > 0 && currentList.getLast().getIdOfSegmentInformation() == previousSegmentInfo.getId()) {
                currentList.getLast().setNewPhaseSegmentInfo(previousSegmentInfo);
            } else { //otherwise, just add the current historical segment onto the end
                currentList.add(new ActuatedSignalVisualizationRepresentation(previousSegmentInfo));
            }
        }
    }

    private void removeOldSegmentDepictionsForRing(List<ActuatedSignalVisualizationRepresentation> visualizations) {
        int index = -1;
        double totalLength = 0;
        double maxLength = getWorkingWidth(DRAW_MIN_WIDTH_PERC, DRAW_MAX_WIDTH_PERC);
        ListIterator<ActuatedSignalVisualizationRepresentation> lit = visualizations.listIterator(visualizations.size());
        double secondsPerPixel = getSecondsPerPixelScale(maxLength, secondsTimeWindowToSupport);

        //figure out which elements fall out
        while (lit.hasPrevious()) {
            ActuatedSignalVisualizationRepresentation visual = lit.previous();
            if (totalLength < maxLength) {
                totalLength += visual.getWidthUnadjusted() / secondsPerPixel;
            } else {
                index = lit.previousIndex();
                break;
            }
        }

        //remove the elements that have totally fallen out
        for (int i = 0; i < index; ++i) {
            visualizations.remove(0);
        }
    }

    private double getBeginningOfXWorkingSpace() {
        return getSize().width * DRAW_MIN_WIDTH_PERC;
    }

    private double getEndOfSpecificWorkingSpace(double perc) {
        return getSize().width * perc;
    }

    private double getBeginningOfYWorkingSpace() {
        return getSize().height * DRAW_MIN_HEIGHT_MOD_PERC;
    }

    private double getBeginningOfPhaseDrawingYWorkingSpace() {
        return getSize().height * DRAW_PHASE_HISTORY_MIN_HEIGHT_MOD_PERC;
    }

    private double getBeginningOfRingAndBarrierYWorkingSpace() {
        return getSize().height * DRAW_HISTORY_RB_BARRIER_MOD_PERC;
    }

    private double getEndOfYWorkingSpace() {
        return getSize().height * DRAW_MAX_HEIGHT_MOD_PERC;
    }

    private double getHistoryRingHeight() {
        if (rb == null) {
            return 0;
        } else {
            return Math.min(getWorkingHeight(DRAW_MIN_HEIGHT_MOD_PERC, DRAW_HISTORY_RB_BARRIER_MOD_PERC) / rb.getNumberOfRings(), maxRingHeight);
        }
    }

    private double getRBDiagramRingHeight() {
        return getWorkingHeight(DRAW_HISTORY_RB_BARRIER_MOD_PERC, DRAW_MAX_HEIGHT_MOD_PERC) / turnDirectionsByPhases.size();
    }

    @Override
    public void paintComponent(Graphics g) {
        double ringHeight = getHistoryRingHeight();
        double beginningOfYPhaseWorkingSpace = getBeginningOfPhaseDrawingYWorkingSpace();
        double beginningOfXWorkingSpace = getBeginningOfXWorkingSpace();
        double endOfRBWorkingSpaceInXDim = getEndOfSpecificWorkingSpace(DRAW_MAX_WIDTH_PERC);
        double currentX = endOfRBWorkingSpaceInXDim;
        double currentY;
        int ringCount = 0;

        ArrayList<LinkedList<ActuatedSignalVisualizationRepresentation>> segmentDepictionsGraphicsCopy = null;
        ArrayList<LimitedPairImplementation<Long, Color>> segmentIdToColorMapsSortedByRing = new ArrayList<LimitedPairImplementation<Long, Color>>();
        //the simulation doesn't stop, and we can't modify the simulation's objects while it is modifying them
        synchronized (mutex) {
            if (clearAll) {
                clearAll = false;
                g.clearRect(0, 0, getSize().width, getSize().height);
            }
            if (phaseSegmentDepictions != null) {
                segmentDepictionsGraphicsCopy = new ArrayList<LinkedList<ActuatedSignalVisualizationRepresentation>>();
                for (LinkedList<ActuatedSignalVisualizationRepresentation> ringList : phaseSegmentDepictions) {
                    segmentDepictionsGraphicsCopy.add(new LinkedList<ActuatedSignalVisualizationRepresentation>(ringList));
                }
            }
        }

        //look through every ring to draw the signal history and determine which signal is is what color
        if (segmentDepictionsGraphicsCopy != null) {
            for (LinkedList<ActuatedSignalVisualizationRepresentation> ringList : segmentDepictionsGraphicsCopy) {
                currentY = ringCount * ringHeight + beginningOfYPhaseWorkingSpace;
                //look through every phase segment
                ListIterator<ActuatedSignalVisualizationRepresentation> visLit = ringList.listIterator(ringList.size());
                while (visLit.hasPrevious()) {
                    boolean isSegmentTheMostRecentSegment = visLit.previousIndex() == ringList.size() - 1;
                    ActuatedSignalVisualizationRepresentation visual = visLit.previous();

                    if (isSegmentTheMostRecentSegment) {
                        segmentIdToColorMapsSortedByRing.add(new LimitedPairImplementation<Long, Color>(visual.getIdOfSegmentInformation(), visual.getPrimarySegmentColor()));
                    }
                    //look through every colored element (i.e., min time and extensions) of the phase segment
                    List<Rectangle2D.Double> rectanglesForRow = visual.getPhaseSegmentRectangles(currentX, currentY, beginningOfXWorkingSpace, ringHeight, getSecondsPerPixelScale(getWorkingWidth(DRAW_MIN_WIDTH_PERC, DRAW_MAX_WIDTH_PERC), secondsTimeWindowToSupport));
                    int index;
                    ListIterator<Rectangle2D.Double> lit = rectanglesForRow.listIterator(rectanglesForRow.size());
                    while (lit.hasPrevious()) {
                        index = lit.previousIndex();
                        Rectangle2D.Double rect = lit.previous();
                        g.setColor(visual.getColorForSegmentAtIndex(index));
                        //Math.ceil to eliminate gaps on the timeline.
                        //g.fillRect((int) Math.round(rect.getX()), (int) Math.round(rect.getY()), (int) Math.round(rect.getWidth()), (int) Math.round(rect.getHeight()));
                        g.fillRect((int) rect.getX(), (int) rect.getY(), (int) rect.getWidth(), (int) rect.getHeight());
                        currentX = Math.max(rect.getX(), beginningOfXWorkingSpace);
                    }
                }
                ++ringCount;
                currentX = endOfRBWorkingSpaceInXDim;
            }
            drawSignalRingAndBarrier(segmentIdToColorMapsSortedByRing, g, beginningOfXWorkingSpace);
            drawGapoutCountdown(g);
            drawRuler(g);
        }

        //nothing to draw, clear it all
        if (ringCount == 0) {
            g.clearRect(0, 0, getSize().width, getSize().height);
        }
    }

    private void drawRuler(Graphics g) {
        double textSpacer = 2;

        double beginningOfYWorkingSpace = getBeginningOfYWorkingSpace();
        double endOfRulerY = getBeginningOfPhaseDrawingYWorkingSpace();
        double beginningOfXWorkingSpace = getBeginningOfXWorkingSpace();
        double currentX = beginningOfXWorkingSpace;
        double xWidth = getWorkingWidth(DRAW_MIN_WIDTH_PERC, DRAW_MAX_WIDTH_PERC);
        double totalTicks = Math.floor(secondsTimeWindowToSupport / MINOR_RULER_TICKS_EVERY_SECONDS);
        double pixelsPerTick = xWidth / totalTicks;
        currentX -= pixelsPerTick;

        g.setColor(RULER_COLOR);

        FontMetrics fontInfo = g.getFontMetrics();
        double textYAdjust = (fontInfo.getHeight() - fontInfo.getDescent() - fontInfo.getLeading());
        for (int i = 0; i <= totalTicks; ++i) {
            currentX += pixelsPerTick;
            if (i % MAJOR_RULER_TICKS_EVERY_SECONDS == 0) {
                int count = (int) (totalTicks - i);
                String output = (count == 0 ? "" : "-") + count;
                double textXAdjust = fontInfo.stringWidth(output) / 2;
                g.drawString(output, (int) (currentX - textXAdjust), (int) (beginningOfYWorkingSpace+textYAdjust));
                g.drawLine((int) currentX, (int) (beginningOfYWorkingSpace+textYAdjust+textSpacer), (int) currentX, (int) endOfRulerY);
            } else if (MAJOR_RULER_TICKS_EVERY_SECONDS != MINOR_RULER_TICKS_EVERY_SECONDS && (MAJOR_RULER_TICKS_EVERY_SECONDS & (int) 1) == 0 && i % (MAJOR_RULER_TICKS_EVERY_SECONDS / 2) == 0) {
                //if major ticks is even using a bitwise or, and minor ticks doesn't == it, draw a tick right in the middle slightly higher
                g.drawLine((int) currentX, (int) (beginningOfYWorkingSpace + (endOfRulerY - beginningOfYWorkingSpace) / 1.5), (int) currentX, (int) endOfRulerY);
            } else {
                g.drawLine((int) currentX, (int) (beginningOfYWorkingSpace + (endOfRulerY - beginningOfYWorkingSpace)/1.2), (int) currentX, (int) endOfRulerY);
            }
        }
    }

    private void drawGapoutCountdown(Graphics g) {
        double ringHeight = getRBDiagramRingHeight();
        double currentY = getBeginningOfRingAndBarrierYWorkingSpace();
        double workingWidth = getWorkingWidth(RING_AND_BARRIER_MAX_DRAW_WIDTH_PERC, DRAW_MAX_WIDTH_PERC);
        double startingX = getEndOfSpecificWorkingSpace(RING_AND_BARRIER_MAX_DRAW_WIDTH_PERC);

        for (int ringNumber = 0; ringNumber < phaseSegmentDepictions.size(); ++ringNumber) {
            ActuatedSignalVisualizationRepresentation asvr = phaseSegmentDepictions.get(ringNumber).getLast();
            double counter = asvr.getPhaseSegmentTimeSinceEpoch();
            double countTo = asvr.getPhaseSegmentLengthTime();
            String counterPad = (counter < 10 ? "0" : "");
            String countToPad = (countTo < 10 ? "0" : "");
            String output = counterPad + String.format("%.2f", counter) + "/" + countToPad + String.format("%2.2f", countTo);
            g.setColor(asvr.getPrimarySegmentColor());

            FontMetrics fontInfo = g.getFontMetrics();
            double textYAdjust = (ringHeight - (fontInfo.getHeight() - fontInfo.getDescent() - fontInfo.getLeading())) / 2;
            double textXAdjust = (workingWidth - fontInfo.stringWidth(output)) / 2;
            g.drawString(output, (int) (startingX + textXAdjust), (int) (currentY + textYAdjust));
            currentY += ringHeight;
        }
    }

    private void drawSignalRingAndBarrier(ArrayList<LimitedPairImplementation<Long, Color>> segmentIdToColorMapsSortedByRing, Graphics g, double beginningOfXWorkingSpace) {
        double beginningOfRBDiagramYWorkingSpace = getBeginningOfRingAndBarrierYWorkingSpace();
        double ringHeight = getRBDiagramRingHeight();
        double currentY = beginningOfRBDiagramYWorkingSpace;
        double currentX = beginningOfXWorkingSpace;

        //two lists holding different info for each phase
        ListIterator<LinkedList<Set<Constants.TurnDirection>>> tdListLit = turnDirectionsByPhases.listIterator();
        ListIterator<LinkedList<Double>> entryHeadingListLit = entryHeadingsByPhases.listIterator();
        //list holding info about current color for phases
        ListIterator<LimitedPairImplementation<Long, Color>> segmentIdToColorMapListLit = segmentIdToColorMapsSortedByRing.listIterator();
        while (tdListLit.hasNext() && entryHeadingListLit.hasNext() && segmentIdToColorMapListLit.hasNext()) {
            int ringIndex = tdListLit.nextIndex();
            LinkedList<Set<Constants.TurnDirection>> tdList = tdListLit.next();
            LinkedList<Double> headingList = entryHeadingListLit.next();
            LimitedPairImplementation<Long, Color> segmentIdColorPair = segmentIdToColorMapListLit.next();

            double elementWidth = getWorkingWidth(DRAW_MIN_WIDTH_PERC, RING_AND_BARRIER_MAX_DRAW_WIDTH_PERC) / tdList.size();
            ListIterator<Set<Constants.TurnDirection>> tdSetLit = tdList.listIterator();
            ListIterator<Double> headingLit = headingList.listIterator();
            while (tdSetLit.hasNext() && headingLit.hasNext()) {
                int phaseIndex = tdSetLit.nextIndex();
                Set<Constants.TurnDirection> tdSet = tdSetLit.next();
                double phaseHeading = headingLit.next();

                g.setColor(RB_PHASE_BG_COLOR);
                g.fillRect((int) currentX, (int) currentY, (int) elementWidth, (int) ringHeight);
                Point2D.Double center = new Point2D.Double(currentX + elementWidth / 2, currentY + ringHeight / 2);
                double amountToMoveFromCenter = PERC_OF_CONTAINING_BOX_FOR_TURN_DIRECTIONS * Math.min(elementWidth, ringHeight);

                Color phaseDrawColor = (segmentIdsInPhases.get(ringIndex).get(phaseIndex).contains(segmentIdColorPair.getKey()) ? segmentIdColorPair.getValue() : TURN_DIRECTION_INACTIVE_COLOR);
                for (Constants.TurnDirection td : tdSet) {
                    double rotationRadians = Util.getNewHeadingFromTurnWithQuarterPiRadianTurns(phaseHeading, td);
                    drawTurnDirection(g, center, elementWidth, ringHeight, amountToMoveFromCenter, phaseDrawColor, rotationRadians);
                }
                drawStemImplyingPhaseDirection(g, center, elementWidth, ringHeight, amountToMoveFromCenter, phaseDrawColor, phaseHeading);
                currentX += elementWidth;
            }

            currentX = beginningOfXWorkingSpace;
            currentY += ringHeight;
        }
    }

    private void drawTurnDirection(Graphics g, Point2D.Double center, double boxWidth, double boxHeight, double moveOutFromCenter, Color drawColor, double rotationRadians) {
        int yCoords[] = {(int) (-TRIANGLE_POINT_DISTANCE_PERC * boxHeight), 0, (int) (TRIANGLE_POINT_DISTANCE_PERC * boxHeight)};
        int xCoords[] = {(int) moveOutFromCenter, (int) (TRIANGLE_POINT_DISTANCE_PERC * boxWidth + moveOutFromCenter), (int) moveOutFromCenter};
        double stemOffset = (TRIANGLE_POINT_DISTANCE_PERC * boxHeight) / 2;
        Graphics2D g2d = (Graphics2D) g.create();

        g2d.setColor(drawColor);
        g2d.translate(center.getX(), center.getY());
        g2d.rotate(rotationRadians);
        g2d.fillPolygon(xCoords, yCoords, Math.min(yCoords.length, xCoords.length));

        //draw stem off of triangle to center of box
        g2d.fillRect(0, (int) (-stemOffset), (int) (moveOutFromCenter), (int) (2 * stemOffset));
        g2d.dispose();
    }

    private void drawStemImplyingPhaseDirection(Graphics g, Point2D.Double center, double boxWidth, double boxHeight, double moveOutFromCenter, Color drawColor, double rotationRadians) {
        double stemOffset = (TRIANGLE_POINT_DISTANCE_PERC * boxHeight) / 2;
        Graphics2D g2d = (Graphics2D) g.create();

        g2d.setColor(drawColor);
        g2d.translate(center.getX(), center.getY());
        g2d.rotate(rotationRadians);

        //draw stem off of triangle to center of box
        g2d.fillRect(-(int) (moveOutFromCenter), (int) (-stemOffset), (int) (moveOutFromCenter + stemOffset), (int) (2 * stemOffset));
        g2d.dispose();
    }

    /**
     * This method is called from within the constructor to initialize the form.
     * WARNING: Do NOT modify this code. The content of this method is always
     * regenerated by the Form Editor.
     */
    @SuppressWarnings("unchecked")
    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(this);
        this.setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 400, Short.MAX_VALUE)
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 300, Short.MAX_VALUE)
        );
    }// </editor-fold>//GEN-END:initComponents


    // Variables declaration - do not modify//GEN-BEGIN:variables
    // End of variables declaration//GEN-END:variables
}
