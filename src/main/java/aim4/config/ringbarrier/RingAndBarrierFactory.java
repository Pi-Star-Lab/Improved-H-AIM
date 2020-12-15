package aim4.config.ringbarrier;

import aim4.config.GreenPhaseData;
import aim4.config.SimConfig;
import aim4.config.TrafficSignal;
import aim4.map.GridMap;
import aim4.map.Road;
import aim4.util.Util;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

/**
 *
 * @author Aaron Parks-Young
 */
public class RingAndBarrierFactory {

    private static final String RING_TAG = "ring";
    private static final String BARRIER_TAG = "barrier";
    private static final String BARRIER_ID_ATTRIBUTE = "id";
    private static final String GREEN_LIGHT_TAG = "green";
    private static final String YELLOW_LIGHT_TAG = "yellow";
    private static final String RED_LIGHT_TAG = "red";
    private static final int EXPECTED_ELEMENTS_IN_PHASE = 5;
    private static final int EXPECTED_ELEMENTS_IN_SHORT_PHASE = 3;
    private static final int EXPECTED_ELEMENTS_IN_BARRIER = 2;
    private static final String CROSS_TURN_CHAR = "C";
    private static final String WITH_FLOW_TURN_CHAR = "F";
    private static final String THROUGH_CHAR = "T";
    private static final String HOLD_UNTIL_OTHER_PHASE_IS_DONE_CHAR = "*";
    private static final String OTHER_PHASES_IN_SAME_INDEX_WAIT_ON_CHAR = "^";

    public static RingAndBarrier createRingAndBarrierFromFile(GridMap map, String filepath) {

        HashMap<Character, Road> roadMapping = setupKeyMapping(map);

        try {
            //open XML and read root tag
            DocumentBuilderFactory dbfact = DocumentBuilderFactory.newInstance();
            DocumentBuilder dbuild = dbfact.newDocumentBuilder();
            Document doc = dbuild.parse(new File(filepath));
            doc.getDocumentElement().normalize();

            Element root = doc.getDocumentElement(); //only expects 1 per file
            return new RingAndBarrier(false, parseRings(root, parseBarriers(root), roadMapping)); //actually do processing of XML
        } catch (ParserConfigurationException ex) {
            Logger.getLogger(RingAndBarrierFactory.class.getName()).log(Level.SEVERE, null, ex);
        } catch (IOException ex) {
            Logger.getLogger(RingAndBarrierFactory.class.getName()).log(Level.SEVERE, null, ex);
        } catch (org.xml.sax.SAXException ex) {
            Logger.getLogger(RingAndBarrierFactory.class.getName()).log(Level.WARNING, "Couldn't read signal file as an XML expected, attempting legacy format read.");
            //wasn't a valid xml.....maybe try reading as if it were a legacy signal file?
            RingAndBarrier legacyCompat = legacyReadFileAndCreateRB(roadMapping, filepath);
            return legacyCompat;
            //Logger.getLogger(RingAndBarrierFactory.class.getName()).log(Level.SEVERE, null, ex);
        }
        return null;
    }

    private static LinkedList<RBRing> parseRings(Element rootelem, HashMap<String, RBPhaseBarrier[]> barriers, HashMap<Character, Road> roadMapping) {
        LinkedList<RBRing> rings = new LinkedList<RBRing>();
        //sort of a master and slave relationship for certain phases in terms of timing. If I had thought about this being a thing, I could have done the barrier entry relationship this way. Whooooooops. It would have been simpler.
        HashMap<Integer, Set<RBPhaseSegment>> segmentsToWait = new HashMap<Integer, Set<RBPhaseSegment>>();
        HashMap<Integer, RBPhaseSegment> segmentsToBeWaitedOn = new HashMap<Integer, RBPhaseSegment>();

        NodeList nodeList = rootelem.getElementsByTagName(RING_TAG);
        ArrayList<String> barrierNamesInOrder = new ArrayList<String>();
        HashSet<String> barrierNames = new HashSet<String>();
        for (int i = 0; i < nodeList.getLength(); i++) {
            ListIterator<String> barrierNamesInOrderIterator = barrierNamesInOrder.listIterator();

            LinkedList<RBPhaseSegment> phases = new LinkedList<RBPhaseSegment>();
            NodeList phaseNodeList = ((Element) nodeList.item(i)).getChildNodes();
            int barrierCount = 0;
            int indexForRing = 0;
            for (int k = 0; k < phaseNodeList.getLength(); k++) {
                segmentsToWait.put(k, new HashSet<RBPhaseSegment>()); //todo, this assumes all things waiting on others will have the same index in the ring and as the thing they're waiting on, which may not be true in all combinations. But this is kind of a secret feature, anyway....
                Node phaseNode = phaseNodeList.item(k);
                if (phaseNode.getNodeType() == Node.ELEMENT_NODE) {
                    if (phaseNode.getNodeName() == null) {
                        throw new RuntimeException("Can't process signal tag as the tag name is null.");
                    }

                    if (phaseNode.getNodeName().equals(GREEN_LIGHT_TAG) || phaseNode.getNodeName().equals(YELLOW_LIGHT_TAG) || phaseNode.getNodeName().equals(RED_LIGHT_TAG)) {
                        if (k == phaseNodeList.getLength() - 1) {
                            throw new RuntimeException("The current implementation of the ring and barrier structure requires that all rings end with a barrier.");
                        } else if (!phaseNode.getNodeName().equals(GREEN_LIGHT_TAG) && k == 0) {
                            throw new RuntimeException("The current implementation of the ring and barrier structure requires that all rings begin with a green light.");
                        }
                        phases.add(createPhaseFromText(
                                tagToColor(phaseNode.getNodeName()),
                                phaseNode.getTextContent(),
                                roadMapping,
                                (phases.isEmpty() ? null : phases.peekLast()),
                                (phases.isEmpty() ? null : phases.peekFirst())));
                        if (phases.getLast().getHoldForOtherPhase()) {
                            segmentsToWait.get(indexForRing++).add(phases.getLast());
                        } else if (phases.getLast().getOtherSegmentsMightHold()) {
                            if (segmentsToBeWaitedOn.putIfAbsent(indexForRing++, phases.getLast()) != null) {
                                throw new RuntimeException("Multiple phases being waiting on with same index in their respective rings. This is currently not allowed.");
                            }
                        }
                    } else if (phaseNode.getNodeName().equals(BARRIER_TAG)) {
                        if (k == 0) {
                            throw new RuntimeException("The current implementation of the ring and barrier structure requires that all rings begin with a green light.");
                        }
                        Node barrierNameNode = phaseNode.getAttributes().getNamedItem(BARRIER_ID_ATTRIBUTE);
                        if (barrierNameNode != null) {
                            String barrierName = barrierNameNode.getTextContent();
                            if (barrierName != null && !barrierName.equals("")) {
                                RBPhaseSegment[] barrier = barriers.get(barrierName);
                                if (barrier == null) {
                                    throw new RuntimeException("Barrier #" + k + " in ring " + i + " couldn't be processed as no barrier with the \"id\" attribute \"" + barrierName + "\" was defined.");
                                } else {
                                    if (i == 0) { //first run through
                                        if (barrierNames.contains(barrierName)) {
                                            throw new RuntimeException("Ring had multiple barriers defined by the same ID, this isn't allowed: " + barrierName);
                                        }

                                        barrierNames.add(barrierName);
                                        barrierNamesInOrder.add(barrierName);
                                    } else {
                                        String name = barrierNamesInOrderIterator.next();
                                        if (name == null ? barrierName != null : !name.equals(barrierName)) {
                                            throw new RuntimeException("Barrier that was expected due to ordering in other rings was: " + name + ". Barrier name found was: " + barrierName + ". All rings must cross barriers in the same order with the current implementation.");
                                        }
                                    }

                                    RBPhaseSegment segYellow = new RBPhaseBarrier(barrier[0]);
                                    errorIfPhaseSegmentTooSmall(segYellow);
                                    RBPhaseSegment segRed = new RBPhaseBarrier(barrier[1]);
                                    errorIfPhaseSegmentTooSmall(segRed);
                                    if (phases.isEmpty()) {
                                        setPreviousAndNextSegments(segYellow, segRed, segRed);
                                        setPreviousAndNextSegments(segRed, segYellow, segYellow);
                                    } else {
                                        setPreviousAndNextSegments(segYellow, phases.peekLast(), segRed);
                                        setPreviousAndNextSegments(segRed, segYellow, phases.peekFirst());
                                    }
                                    phases.add(segYellow);
                                    phases.add(segRed);
                                }
                            } else {
                                throw new RuntimeException("Barrier #" + barrierCount + " in ring " + i + " couldn't be processed as no valid name was given for the \"id\" attribute. Content \"id\" attribute was: " + (barrierName == null ? "null" : "\"" + barrierName + "\""));
                            }
                        } else {
                            throw new RuntimeException("Barrier #" + barrierCount + " in ring " + i + " couldn't be processed as no \"id\" attribute was present. Content of barrier was: \"" + phaseNode.getTextContent() + "\"");
                        }
                    } else {
                        throw new RuntimeException("Can't process signal tag as the tag name is an unexpected value: " + phaseNode.getNodeName());
                    }
                    ++barrierCount;
                }
            }
            rings.add(new RBRing(phases, SimConfig.SIM_ALLOWS_EARLY_GAPOUT));
        }

        for (Integer i : segmentsToBeWaitedOn.keySet()) {
            for (RBPhaseSegment waitingSeg : segmentsToWait.get(i)) {
                segmentsToBeWaitedOn.get(i).addSegmentHolding(waitingSeg);
                waitingSeg.setSegmentToHoldOn(segmentsToBeWaitedOn.get(i));
            }
        }

        return rings;
    }

    private static HashMap<String, RBPhaseBarrier[]> parseBarriers(Element rootelem) {
        HashMap<String, RBPhaseBarrier[]> barriers = new HashMap<String, RBPhaseBarrier[]>();
        NodeList nodeList = rootelem.getElementsByTagName(BARRIER_TAG);

        int barrierDefinitionCount = 0; //count of which barrier definition we're on
        for (int i = 0; i < nodeList.getLength(); i++) {
            Node barrierNode = nodeList.item(i);
            if (barrierNode.getParentNode() == rootelem) { //skip over barriers embedded in rings
                Node barrierNameNode = barrierNode.getAttributes().getNamedItem(BARRIER_ID_ATTRIBUTE);
                if (barrierNameNode != null) {
                    String barrierName = barrierNameNode.getTextContent();
                    if (barrierName != null && !barrierName.equals("")) {
                        barriers.put(barrierName, createBarrierFromText(barrierNode.getTextContent()));
                    } else {
                        throw new RuntimeException("Barrier #" + barrierDefinitionCount + " couldn't be processed as no valid name was given for the \"id\" attribute. Content \"id\" attribute was: " + (barrierName == null ? "null" : "\"" + barrierName + "\""));
                    }
                } else {
                    throw new RuntimeException("Barrier #" + barrierDefinitionCount + " couldn't be processed as no \"id\" attribute was present. Content of barrier was: \"" + barrierNode.getTextContent() + "\"");
                }
                ++barrierDefinitionCount;
            }
        }
        return barriers;
    }

    private static TrafficSignal tagToColor(String tag) {
        if (tag == null) {
            throw new RuntimeException("Can't process signal color as the tag is null.");
        }

        if (tag.equals(GREEN_LIGHT_TAG)) {
            return TrafficSignal.GREEN;
        } else if (tag.equals(YELLOW_LIGHT_TAG)) {
            return TrafficSignal.YELLOW;
        } else if (tag.equals(RED_LIGHT_TAG)) {
            return TrafficSignal.RED;
        } else {
            throw new RuntimeException("Can't process signal color as tag is an unexpected value: " + tag);
        }
    }

    private static RBPhaseSegment createPhaseFromText(TrafficSignal color, String info, HashMap<Character, Road> roadMapping, RBPhaseSegment previous, RBPhaseSegment next) {
        String[] tokens = info.split(",");
        if (tokens.length != EXPECTED_ELEMENTS_IN_PHASE && tokens.length != EXPECTED_ELEMENTS_IN_SHORT_PHASE) {

            throw new RuntimeException("Expected " + EXPECTED_ELEMENTS_IN_PHASE + " or " + EXPECTED_ELEMENTS_IN_SHORT_PHASE + " elements in phase. Found " + tokens.length + ". Please check the integrity of the signal file.\n" + info);
        }
        //clean tokens so no whitespace exists using regex replacement
        for (int i = 0; i < tokens.length; i++) {
            tokens[i] = tokens[i].replaceAll("\\s", "");
        }

        try {
            if (tokens[0].length() == 1 && tokens.length == EXPECTED_ELEMENTS_IN_PHASE && roadMapping.containsKey(tokens[0].charAt(0))) {
                RBPhaseSegment seg = new RBPhaseSegment(roadMapping.get(tokens[0].charAt(0)),
                        Double.parseDouble(tokens[2]),
                        Double.parseDouble(tokens[3]),
                        Double.parseDouble(tokens[4]),
                        color,
                        tokens[1].toUpperCase().contains(CROSS_TURN_CHAR),
                        tokens[1].toUpperCase().contains(THROUGH_CHAR),
                        tokens[1].toUpperCase().contains(THROUGH_CHAR),//tokens[1].toUpperCase().contains(WITH_FLOW_TURN_CHAR),
                        tokens[1].toUpperCase().contains(HOLD_UNTIL_OTHER_PHASE_IS_DONE_CHAR),
                        tokens[1].toUpperCase().contains(OTHER_PHASES_IN_SAME_INDEX_WAIT_ON_CHAR)
                );
                errorIfPhaseSegmentTooSmall(seg);
                setPreviousAndNextSegments(seg, previous, next);
                return seg;
            } else if (tokens[0].length() == 1 && tokens.length == EXPECTED_ELEMENTS_IN_SHORT_PHASE && roadMapping.containsKey(tokens[0].charAt(0))) {
                RBPhaseSegment seg = new RBPhaseSegment(roadMapping.get(tokens[0].charAt(0)),
                        Double.parseDouble(tokens[2]),
                        color,
                        tokens[1].toUpperCase().contains(CROSS_TURN_CHAR),
                        tokens[1].toUpperCase().contains(THROUGH_CHAR),
                        tokens[1].toUpperCase().contains(THROUGH_CHAR),//tokens[1].toUpperCase().contains(WITH_FLOW_TURN_CHAR),
                        tokens[1].toUpperCase().contains(HOLD_UNTIL_OTHER_PHASE_IS_DONE_CHAR),
                        tokens[1].toUpperCase().contains(OTHER_PHASES_IN_SAME_INDEX_WAIT_ON_CHAR)
                );
                errorIfPhaseSegmentTooSmall(seg);
                setPreviousAndNextSegments(seg, previous, next);
                return seg;
            } else {
                throw new Exception();
            }
        } catch (Exception e) {
            throw new RuntimeException("Unable to translate line into phase: " + info + "\n" + e);
        }
    }

    private static void setPreviousAndNextSegments(RBPhaseSegment targetSegment, RBPhaseSegment previousSegment, RBPhaseSegment nextSegment) {
        if (previousSegment != null) {
            previousSegment.setNextPhaseSegment(targetSegment);
            targetSegment.setPreviousPhaseSegment(previousSegment);
        } else {
            targetSegment.setPreviousPhaseSegment(targetSegment);
        }

        if (nextSegment != null) {
            nextSegment.setPreviousPhaseSegment(targetSegment);
            targetSegment.setNextPhaseSegment(nextSegment);
        } else {
            targetSegment.setNextPhaseSegment(targetSegment);
        }
    }

    private static RBPhaseBarrier[] createBarrierFromText(String info) {
        String[] tokens = info.split(",");
        if (tokens.length != EXPECTED_ELEMENTS_IN_BARRIER) {
            throw new RuntimeException("Expected " + EXPECTED_ELEMENTS_IN_BARRIER + " elements in barrier. Found " + tokens.length + ". Please check the integrity of the signal file.\n" + info);
        }

        //clean tokens so no whitespace exists using regex replacement
        for (int i = 0; i < tokens.length; i++) {
            tokens[i] = tokens[i].replaceAll("\\s", "");
        }

        try {
            RBPhaseBarrier[] barrier = new RBPhaseBarrier[2];
            barrier[0] = new RBPhaseBarrier(Double.parseDouble(tokens[0]), TrafficSignal.YELLOW);
            barrier[1] = new RBPhaseBarrier(Double.parseDouble(tokens[1]), TrafficSignal.RED);
            return barrier;
        } catch (Exception e) {
            throw new RuntimeException("Unable to translate line into barrier: " + info);
        }
    }

    private static HashMap<Character, Road> setupKeyMapping(GridMap map) {
        HashMap<Character, Road> keyMapping = new HashMap<Character, Road>();
        for (Road road : map.getRoads()) {
            if (road.getName().equals("1st Avenue N")) {
                keyMapping.put('N', road);
            } else if (road.getName().equals("1st Avenue S")) {
                keyMapping.put('S', road);
            } else if (road.getName().equals("1st Street E")) {
                keyMapping.put('E', road);
            } else if (road.getName().equals("1st Street W")) {
                keyMapping.put('W', road);
            }
        }
        return keyMapping;
    }

    private static RingAndBarrier legacyReadFileAndCreateRB(HashMap<Character, Road> roadMapping, String filepath) {
        List<String> strs = null;
        try {
            strs = Util.readFileToStrArray(filepath);
        } catch (IOException e) {
            System.err.println("Error: " + e.getMessage());
        }
        if (strs != null) {
            return processLegacyData(roadMapping, strs);
        } else {
            return null;
        }
    }

    private static RingAndBarrier processLegacyData(HashMap<Character, Road> roadMapping, List<String> strs) {
        ArrayList<RBRing> rings = new ArrayList<RBRing>();

        ArrayList<LinkedList<RBPhaseSegment>> listsForRings = new ArrayList<LinkedList<RBPhaseSegment>>();
        HashMap<Road, Integer> roadToRingMap = new HashMap<Road, Integer>();
        int count = 0;
        for (char c : roadMapping.keySet()) {
            listsForRings.add(new LinkedList<RBPhaseSegment>());
            roadToRingMap.put(roadMapping.get(Character.toUpperCase(c)), count++);
        }

        //skip header line, go through every other line
        for (int i = 1; i < strs.size(); i++) {
            String[] tokens = strs.get(i).split(",");
            //clean tokens so no whitespace exists using regex replacement
            for (int k = 0; k < tokens.length; k++) {
                tokens[k] = tokens[k].replaceAll("\\s", "");
            }

            HashSet<Character> remainingChars = new HashSet<Character>(roadMapping.keySet());

            for (int j = 0; j < tokens[0].length(); ++j) {
                if (roadMapping.keySet().contains(Character.toUpperCase(tokens[0].charAt(j)))) {
                    //this is included because position of the key relative to a * now matters to determine if it's a permissive phase, and having multiple of the same direction in a phase should error
                    if (!remainingChars.contains(Character.toUpperCase(tokens[0].charAt(j)))) {
                        throw new RuntimeException("Key \"" + Character.toUpperCase(tokens[0].charAt(j)) + "\" appears multiple times in phase. This is not allowed.\n" + strs.get(i));
                    }
                    remainingChars.remove(Character.toUpperCase(tokens[0].charAt(j)));

                    Road rd = roadMapping.get(Character.toUpperCase(tokens[0].charAt(j)));

                    double greenPhase;
                    if (GreenPhaseData.on) {
                        // in this case, we use our generated green phase length
                        // this is used for experiment of finding best green phase length
                        greenPhase = GreenPhaseData.getGreenPhase(i - 1); // be careful, i starts at 1 
                    } else {
                        // the duration of the green signal
                        greenPhase = Double.parseDouble(tokens[1]);
                    }

                    boolean crossTurn = false;
                    if (j != tokens[0].length() - 1 && tokens[0].charAt(j + 1) == '*') {
                        crossTurn = true;
                        ++j;
                    }

                    LinkedList<RBPhaseSegment> innerList = listsForRings.get(roadToRingMap.get(rd));
                    RBPhaseSegment segGreen = new RBPhaseSegment(rd, greenPhase, TrafficSignal.GREEN, crossTurn, true, true, false, false);
                    errorIfPhaseSegmentTooSmall(segGreen);
                    RBPhaseSegment segYellow = new RBPhaseBarrier(Double.parseDouble(tokens[2]), TrafficSignal.YELLOW);
                    errorIfPhaseSegmentTooSmall(segYellow);
                    RBPhaseSegment segRed = new RBPhaseBarrier(Double.parseDouble(tokens[3]), TrafficSignal.RED);
                    errorIfPhaseSegmentTooSmall(segRed);
                    if (innerList.isEmpty()) {
                        setPreviousAndNextSegments(segGreen, segRed, segYellow);
                        setPreviousAndNextSegments(segYellow, segGreen, segRed);
                        setPreviousAndNextSegments(segRed, segYellow, segGreen);
                    } else {
                        setPreviousAndNextSegments(segGreen, innerList.peekLast(), segYellow);
                        setPreviousAndNextSegments(segYellow, segGreen, segRed);
                        setPreviousAndNextSegments(segRed, segYellow, innerList.peekFirst());
                    }

                    innerList.add(segGreen);
                    innerList.add(segYellow); //barrier yellow
                    innerList.add(segRed); //barrier red
                } else {
                    throw new RuntimeException("When attempting legacy read of signal, failed to map character to road: " + tokens[0].charAt(j));
                }
            }
            for (char c : remainingChars) {
                double greenPhase;
                if (GreenPhaseData.on) {
                    // in this case, we use our generated green phase length
                    // this is used for experiment of finding best green phase length
                    greenPhase = GreenPhaseData.getGreenPhase(i - 1); // be careful, i starts at 1 
                } else {
                    // the duration of the green signal
                    greenPhase = Double.parseDouble(tokens[1]);
                }

                Road rd = roadMapping.get(Character.toUpperCase(c));
                LinkedList<RBPhaseSegment> innerList = listsForRings.get(roadToRingMap.get(rd));
                RBPhaseSegment seg1 = new RBPhaseSegment(rd, greenPhase, TrafficSignal.RED, true, true, true, false, false);
                errorIfPhaseSegmentTooSmall(seg1);
                RBPhaseSegment seg2 = new RBPhaseBarrier(Double.parseDouble(tokens[2]), TrafficSignal.RED);
                errorIfPhaseSegmentTooSmall(seg2);
                RBPhaseSegment seg3 = new RBPhaseBarrier(Double.parseDouble(tokens[3]), TrafficSignal.RED);
                errorIfPhaseSegmentTooSmall(seg3);
                if (innerList.isEmpty()) {
                    setPreviousAndNextSegments(seg1, seg3, seg2);
                    setPreviousAndNextSegments(seg2, seg1, seg3);
                    setPreviousAndNextSegments(seg3, seg2, seg1);
                } else {
                    setPreviousAndNextSegments(seg1, innerList.peekLast(), seg2);
                    setPreviousAndNextSegments(seg2, seg1, seg3);
                    setPreviousAndNextSegments(seg3, seg2, innerList.peekFirst());
                }

                seg1.setLegacyIsPlaceholder(true);
                seg2.setLegacyIsPlaceholder(true);
                seg3.setLegacyIsPlaceholder(true);

                innerList.add(seg1);
                innerList.add(seg2);
                innerList.add(seg3);
            }
        }

        for (LinkedList<RBPhaseSegment> listForRings : listsForRings) {
            rings.add(new RBRing(listForRings, SimConfig.SIM_ALLOWS_EARLY_GAPOUT));
        }

        return new RingAndBarrier(true, rings);
    }

    private static void errorIfPhaseSegmentTooSmall(RBPhaseSegment seg) {
        if (seg.getMinTimeAssumingNoSegmentToHoldOnTo() <= SimConfig.TIME_STEP && seg.getMinTimeAssumingNoSegmentToHoldOnTo() != 0) {
            throw new RuntimeException("The timestep for the experiment was set to " + SimConfig.TIME_STEP + "s while at least one phase segment had a minimum length of " + seg.getMinTimeAssumingNoSegmentToHoldOnTo() + "s. The lengths of all phase segments must be larger than the timestep for proper operation of the ring and barrier.");
        }
    }
}
