package aim4.gui.frame.signalvisualization;

import aim4.config.TrafficSignal;
import aim4.config.ringbarrier.HistoricalRBSegmentInformation;
import java.awt.Color;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

public class ActuatedSignalVisualizationRepresentation {

    private static final Color GREEN_COLOR = new Color(24, 150, 19);
    private static final Color GREEN_EXTENSION_COLOR = new Color(173, 255, 47);
    private static final Color GREEN_WAIT_EXTENSION_COLOR = new Color(0, 255, 215);
    private static final Color YELLOW_COLOR = Color.YELLOW;
    private static final Color RED_COLOR = Color.RED;
    private static final Color ERROR_COLOR_UNEXPECTED_GAP_EXTENSION = Color.CYAN;
    private static final Color ERROR_COLOR_UNEXPECTED_WAITING_EXTENSION = Color.BLUE;
    private static final Color ERROR_COLOR_GENERAL = Color.PINK;

    private final Object phaseSegmentRectanglesMutex = new Object(); //mutex to keep thread concurrent modification error from happening
    private ArrayList<Rectangle2D.Double> phaseSegmentRectangles;
    private ArrayList<Color> phaseSegmentRectangleColors;
    private HistoricalRBSegmentInformation segInf;
    private double widthUnadjusted;

    public ActuatedSignalVisualizationRepresentation(HistoricalRBSegmentInformation segInf) {
        this.segInf = segInf;
        phaseSegmentRectangles = new ArrayList<Rectangle2D.Double>();
        phaseSegmentRectangleColors = new ArrayList<Color>();
        widthUnadjusted = 0;
        createRectanglesBasedOnPhaseSegmentInfo();
    }

    private void createRectanglesBasedOnPhaseSegmentInfo() {
        synchronized (phaseSegmentRectanglesMutex) {
            if (segInf.getSimTimeWhenUpdated() <= segInf.getSimTimeWhenMinTimeExpires()) {
                //fill in the time that has elapsed short of the min phase time
                phaseSegmentRectangles.add(new Rectangle2D.Double(0, 0, segInf.getTimeElapsedSinceEpoch(), 0));
                phaseSegmentRectangleColors.add(getColorForMinTimeInPhase(segInf.getColor()));
                widthUnadjusted += phaseSegmentRectangles.get(phaseSegmentRectangles.size() - 1).getWidth();
            } else {
                //fill in the min phase time
                phaseSegmentRectangles.add(new Rectangle2D.Double(0, 0, segInf.getMinTime(), 0));
                phaseSegmentRectangleColors.add(getColorForMinTimeInPhase(segInf.getColor()));
                widthUnadjusted += phaseSegmentRectangles.get(phaseSegmentRectangles.size() - 1).getWidth();

                //handle forced extensions due to waiting on a barrier vs simple gap extensions
                if (segInf.getSimTimeWhenUpdated() <= segInf.getSimTimeWhenExpectedEndTimeExpires()) {
                    phaseSegmentRectangles.add(new Rectangle2D.Double(0, 0, segInf.getTimeElapsedSinceEpoch() - segInf.getMinTime(), 0));
                    phaseSegmentRectangleColors.add(getColorForExtendedTimeInPhase(segInf.getColor()));

                    widthUnadjusted += phaseSegmentRectangles.get(phaseSegmentRectangles.size() - 1).getWidth();
                } else {
                    phaseSegmentRectangles.add(new Rectangle2D.Double(0, 0, segInf.getSimTimeWhenExpectedEndTimeExpires() - segInf.getSimTimeWhenMinTimeExpires(), 0));
                    phaseSegmentRectangleColors.add(getColorForExtendedTimeInPhase(segInf.getColor()));

                    widthUnadjusted += phaseSegmentRectangles.get(phaseSegmentRectangles.size() - 1).getWidth();

                    phaseSegmentRectangles.add(new Rectangle2D.Double(0, 0, segInf.getSimTimeWhenUpdated() - segInf.getSimTimeWhenExpectedEndTimeExpires(), 0));
                    phaseSegmentRectangleColors.add(getColorForWaitingExtendedTimeInPhase(segInf.getColor()));

                    widthUnadjusted += phaseSegmentRectangles.get(phaseSegmentRectangles.size() - 1).getWidth();
                }
            }
        }
    }

    public void setNewPhaseSegmentInfo(HistoricalRBSegmentInformation segInf) {
        //ensure the update is valid (same ID, same final variables, same or future time or early gapout is allowed), and update if it is
        synchronized (phaseSegmentRectanglesMutex) {
            if (this.segInf.getId() == segInf.getId())
                    /*&& this.segInf.getEpoch() == segInf.getEpoch()
                    && this.segInf.getMaxTime() == segInf.getMaxTime()
                    && this.segInf.getMinTime() == segInf.getMinTime()
                    && this.segInf.isEarlyGapoutAllowed() == segInf.isEarlyGapoutAllowed()
                    && this.segInf.getSimTimeWhenUpdated() <= segInf.getSimTimeWhenUpdated()
                    && //check whether the end time or gapout time has been pushed back or if early gapouts are allowed and times are valid (this is probably a liiiitle over complicated)
                    ((this.segInf.getSimTimeWhenExpectedEndTimeExpires() <= segInf.getSimTimeWhenExpectedEndTimeExpires() && segInf.getSimTimeWhenExpectedEndTimeExpires() <= segInf.getSimTimeWhenMaxTimeExpires())
                    || (this.segInf.isEarlyGapoutAllowed() == segInf.isEarlyGapoutAllowed() && this.segInf.isEarlyGapoutAllowed() && segInf.getSimTimeWhenExpectedEndTimeExpires() >= segInf.getSimTimeWhenMinTimeExpires() && segInf.getSimTimeWhenExpectedEndTimeExpires() <= segInf.getSimTimeWhenMaxTimeExpires())))*/ {   
                this.segInf = segInf;
                phaseSegmentRectangles = new ArrayList<Rectangle2D.Double>();
                phaseSegmentRectangleColors = new ArrayList<Color>();
                widthUnadjusted = 0;
                createRectanglesBasedOnPhaseSegmentInfo();
            }
        }
    }

    private Color getColorForMinTimeInPhase(TrafficSignal signalEnumValue) {
        switch (signalEnumValue) {
            case RED:
                return RED_COLOR;
            case YELLOW:
                return YELLOW_COLOR;
            case GREEN:
                return GREEN_COLOR;
            default:
                return ERROR_COLOR_GENERAL;
        }
    }

    public Color getPrimarySegmentColor() {
        return getColorForMinTimeInPhase(segInf.getColor());
    }

    public long getIdOfSegmentInformation() {
        synchronized (phaseSegmentRectanglesMutex) {
            if (segInf != null) {
                return segInf.getId();
            } else {
                return HistoricalRBSegmentInformation.INVALID_ID;
            }
        }
    }

    private Color getColorForExtendedTimeInPhase(TrafficSignal signalEnumValue) {
        switch (signalEnumValue) {
            case GREEN:
                return GREEN_EXTENSION_COLOR;
            default:
                return ERROR_COLOR_UNEXPECTED_GAP_EXTENSION;
        }
    }

    private Color getColorForWaitingExtendedTimeInPhase(TrafficSignal signalEnumValue) {
        switch (signalEnumValue) {
            case GREEN:
                return GREEN_WAIT_EXTENSION_COLOR;
            default:
                return ERROR_COLOR_UNEXPECTED_WAITING_EXTENSION;
        }
    }

    public double getPhaseSegmentLengthTime() {
        return segInf.getSimTimeWhenExpectedEndTimeExpires() - segInf.getEpoch();
    }

    public double getPhaseSegmentTimeSinceEpoch() {
        return segInf.getTimeElapsedSinceEpoch();
    }

    /**
     * Gets rectangle representations of the phase segment for drawing.
     *
     * @param endingXOffset the final X coordinate the group of rectangles is
     * allowed to take
     * @param yOffset the y offset at which to draw the rectangles
     * @param lowestXCoordinateAllowed the lowest x coordinate the rectangles
     * are allowed to be drawn to, rectangles that would have extended before
     * this are truncated
     * @param heightOfRow the height of the rectangles
     * @param pixelsPerSecond the scale for the X axis at which to draw the
     * rectangles
     * @return a list of rectangles whose coordinates and dimensions are ready
     * for drawing
     */
    public List<Rectangle2D.Double> getPhaseSegmentRectangles(double endingXOffset, double yOffset, double lowestXCoordinateAllowed, double heightOfRow, double pixelsPerSecond) {
        if (pixelsPerSecond < 0) {
            throw new IllegalArgumentException("pixelsPerSecond cannot be negative, provided value was: " + pixelsPerSecond);
        }
        synchronized (phaseSegmentRectanglesMutex) {
            LinkedList<Rectangle2D.Double> retList = new LinkedList<Rectangle2D.Double>();
            ListIterator<Rectangle2D.Double> lit = phaseSegmentRectangles.listIterator(phaseSegmentRectangles.size());
            double currentXOffset = endingXOffset;
            while (lit.hasPrevious()) {
                Rectangle2D.Double rect = lit.previous();
                double width = rect.getWidth() / pixelsPerSecond;
                // check if any of the rectangle can be drawn
                if (currentXOffset > lowestXCoordinateAllowed) {
                    double xCoord = currentXOffset - width;
                    double xDistanceToLowestXAllowed = (xCoord + width) - lowestXCoordinateAllowed;
                    retList.addFirst(new Rectangle2D.Double(Math.max(lowestXCoordinateAllowed, xCoord), yOffset, Math.min(xDistanceToLowestXAllowed, width), heightOfRow));
                    currentXOffset = xCoord;
                } else {
                    //add a dummy rectangle so the index is correct for color checks for the caller, but so the rectangle is not drawn.
                    retList.addFirst(new Rectangle2D.Double(-1, -1, 0, 0));
                }
            }

            return retList;
        }
    }

    public double getWidthUnadjusted() {
        return widthUnadjusted;
    }

    public Color getColorForSegmentAtIndex(int index) {
        synchronized (phaseSegmentRectanglesMutex) {
            if (index < phaseSegmentRectangleColors.size()) {
                return phaseSegmentRectangleColors.get(index);
            } else {
                return ERROR_COLOR_GENERAL;
            }
        }
    }

}
