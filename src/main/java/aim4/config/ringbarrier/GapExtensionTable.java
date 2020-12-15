package aim4.config.ringbarrier;

import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class GapExtensionTable {

    //table of maximum allowable headway by detection zone length (meters) by avg speed in m/s length to gap extension time
    private static NavigableMap<Double, NavigableMap<Double, NavigableMap<Double, Double>>> table = null;

    public static Double getMaximumGapExtensionInSecondsClosestToProvidedKeys(double maximumAllowableHeadwayInSeconds, double detectionLengthInMeters, double avgApproachSpeedInMetersPerSecond) {
        initializeTableIfNeeded();
        Entry<Double, NavigableMap<Double, NavigableMap<Double, Double>>> preferredOuterEntry = getPreferredOuterEntry(maximumAllowableHeadwayInSeconds, table.floorEntry(maximumAllowableHeadwayInSeconds), table.ceilingEntry(maximumAllowableHeadwayInSeconds));
        if (preferredOuterEntry.getValue() != null) {
            //clipping potentially infinite values so distance from key calculations are safe
            double realDetectionLength = Math.max(Double.MIN_VALUE, Math.min(detectionLengthInMeters, Double.MAX_VALUE)); 
            Entry<Double, NavigableMap<Double, Double>> preferredMidEntry = getPreferredMidEntry(detectionLengthInMeters, preferredOuterEntry.getValue().floorEntry(detectionLengthInMeters), preferredOuterEntry.getValue().ceilingEntry(detectionLengthInMeters));
            if (preferredMidEntry.getValue() != null) {
                Double val = getPreferredInnerEntry(avgApproachSpeedInMetersPerSecond, preferredMidEntry.getValue().floorEntry(avgApproachSpeedInMetersPerSecond), preferredMidEntry.getValue().ceilingEntry(avgApproachSpeedInMetersPerSecond)).getValue();
                if (val == null) {
                    throw new RuntimeException("Inner result is null. This is not allowed.");
                }
                return val;
            } else {
                throw new RuntimeException("Mid map is null. This is not allowed.");
            }
        } else {
            throw new RuntimeException("Outer map is null. This is not allowed.");
        }
    }

    private static Entry<Double, NavigableMap<Double, NavigableMap<Double, Double>>> getPreferredOuterEntry(double keyval, Entry<Double, NavigableMap<Double, NavigableMap<Double, Double>>> entry1, Entry<Double, NavigableMap<Double, NavigableMap<Double, Double>>> entry2) {
        if (entry1 == null || entry2 == null) {
            if (entry1 == null && entry2 != null) {
                return entry2;
            } else if (entry1 != null && entry2 == null) {
                return entry1;
            } else if (entry1 == null && entry2 == null) {
                throw new RuntimeException("Both entries are null. This is not allowed.");
            }
        }

        if (entry1.getKey() == null) {
            throw new RuntimeException("Entry1's key is null. This is not allowed.");
        } else if (entry2.getKey() == null) {
            throw new RuntimeException("Entry2's key is null. This is not allowed.");
        }

        if (Math.abs(entry1.getKey() - keyval) <= Math.abs(entry2.getKey() - keyval)) {
            return entry1;
        } else { // (Math.abs(entry1.getKey() - keyval) > Math.abs(entry2.getKey() - keyval)) {
            return entry2;
        }
    }

    private static Entry<Double, NavigableMap<Double, Double>> getPreferredMidEntry(double keyval, Entry<Double, NavigableMap<Double, Double>> entry1, Entry<Double, NavigableMap<Double, Double>> entry2) {
        if (entry1 == null || entry2 == null) {
            if (entry1 == null && entry2 != null) {
                return entry2;
            } else if (entry1 != null && entry2 == null) {

                return entry1;
            } else if (entry1 == null && entry2 == null) {
                throw new RuntimeException("Both entries are null. This is not allowed.");
            }
        }

        if (entry1.getKey() == null) {
            throw new RuntimeException("Entry1's key is null. This is not allowed.");
        } else if (entry2.getKey() == null) {
            throw new RuntimeException("Entry2's key is null. This is not allowed.");
        }

        if (Math.abs(entry1.getKey() - keyval) <= Math.abs(entry2.getKey() - keyval)) {
            return entry1;
        } else { // (Math.abs(entry1.getKey() - keyval) > Math.abs(entry2.getKey() - keyval)) {
            return entry2;
        }
    }

    private static Entry<Double, Double> getPreferredInnerEntry(double keyval, Entry<Double, Double> entry1, Entry<Double, Double> entry2) {
        if (entry1 == null || entry2 == null) {
            if (entry1 == null && entry2 != null) {
                return entry2;
            } else if (entry1 != null && entry2 == null) {
                return entry1;
            } else if (entry1 == null && entry2 == null) {
                throw new RuntimeException("Both entries are null. This is not allowed.");
            }
        }

        if (entry1.getKey() == null) {
            throw new RuntimeException("Entry1's key is null. This is not allowed.");
        } else if (entry2.getKey() == null) {
            throw new RuntimeException("Entry2's key is null. This is not allowed.");
        }

        if (Math.abs(entry1.getKey() - keyval) <= Math.abs(entry2.getKey() - keyval)) {
            return entry1;
        } else { // (Math.abs(entry1.getKey() - keyval) > Math.abs(entry2.getKey() - keyval)) {
            return entry2;
        }
    }

    private static void initializeTableIfNeeded() {
        if (table == null) {
            table = new TreeMap<Double, NavigableMap<Double, NavigableMap<Double, Double>>>();
            //table 5-10 https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter5.htm
            table.put(3.0, new TreeMap<Double, NavigableMap<Double, Double>>());

            table.get(3.0).put(1.8288, new TreeMap<Double, Double>());
            table.get(3.0).get(1.8288).put(11.176, 2.2);
            table.get(3.0).get(1.8288).put(13.4112, 2.3);
            table.get(3.0).get(1.8288).put(15.6464, 2.4);
            table.get(3.0).get(1.8288).put(17.8816, 2.5);
            table.get(3.0).get(1.8288).put(20.1168, 2.6);

            table.get(3.0).put(4.572, new TreeMap<Double, Double>());
            table.get(3.0).get(4.572).put(11.176, 1.9);
            table.get(3.0).get(4.572).put(13.4112, 2.1);
            table.get(3.0).get(4.572).put(15.6464, 2.2);
            table.get(3.0).get(4.572).put(17.8816, 2.3);
            table.get(3.0).get(4.572).put(20.1168, 2.4);

            table.get(3.0).put(7.62, new TreeMap<Double, Double>());
            table.get(3.0).get(7.62).put(11.176, 1.6);
            table.get(3.0).get(7.62).put(13.4112, 1.8);
            table.get(3.0).get(7.62).put(15.6464, 2.0);
            table.get(3.0).get(7.62).put(17.8816, 2.1);
            table.get(3.0).get(7.62).put(20.1168, 2.2);

            table.get(3.0).put(10.668, new TreeMap<Double, Double>());
            table.get(3.0).get(10.668).put(11.176, 1.3);
            table.get(3.0).get(10.668).put(13.4112, 1.6);
            table.get(3.0).get(10.668).put(15.6464, 1.8);
            table.get(3.0).get(10.668).put(17.8816, 1.9);
            table.get(3.0).get(10.668).put(20.1168, 2.1);

            table.get(3.0).put(13.716, new TreeMap<Double, Double>());
            table.get(3.0).get(13.716).put(11.176, 1.0);
            table.get(3.0).get(13.716).put(13.4112, 1.3);
            table.get(3.0).get(13.716).put(15.6464, 1.6);
            table.get(3.0).get(13.716).put(17.8816, 1.7);
            table.get(3.0).get(13.716).put(20.1168, 1.9);

            table.get(3.0).put(16.764, new TreeMap<Double, Double>());
            table.get(3.0).get(16.764).put(11.176, 0.7);
            table.get(3.0).get(16.764).put(13.4112, 1.1);
            table.get(3.0).get(16.764).put(15.6464, 1.3);
            table.get(3.0).get(16.764).put(17.8816, 1.6);
            table.get(3.0).get(16.764).put(20.1168, 1.7);

            table.get(3.0).put(19.812, new TreeMap<Double, Double>());
            table.get(3.0).get(19.812).put(11.176, 0.4);
            table.get(3.0).get(19.812).put(13.4112, 0.8);
            table.get(3.0).get(19.812).put(15.6464, 1.1);
            table.get(3.0).get(19.812).put(17.8816, 1.4);
            table.get(3.0).get(19.812).put(20.1168, 1.5);

            table.get(3.0).put(22.86, new TreeMap<Double, Double>());
            table.get(3.0).get(22.86).put(11.176, 0.1);
            table.get(3.0).get(22.86).put(13.4112, 0.6);
            table.get(3.0).get(22.86).put(15.6464, 0.9);
            table.get(3.0).get(22.86).put(17.8816, 1.2);
            table.get(3.0).get(22.86).put(20.1168, 1.4);

            //4.0 second allowable headway
            table.put(4.0, new TreeMap<Double, NavigableMap<Double, Double>>());

            table.get(4.0).put(1.8288, new TreeMap<Double, Double>());
            table.get(4.0).get(1.8288).put(11.176, 3.2);
            table.get(4.0).get(1.8288).put(13.4112, 3.3);
            table.get(4.0).get(1.8288).put(15.6464, 3.4);
            table.get(4.0).get(1.8288).put(17.8816, 3.5);
            table.get(4.0).get(1.8288).put(20.1168, 3.6);

            table.get(4.0).put(4.572, new TreeMap<Double, Double>());
            table.get(4.0).get(4.572).put(11.176, 2.9);
            table.get(4.0).get(4.572).put(13.4112, 3.1);
            table.get(4.0).get(4.572).put(15.6464, 3.2);
            table.get(4.0).get(4.572).put(17.8816, 3.3);
            table.get(4.0).get(4.572).put(20.1168, 3.4);

            table.get(4.0).put(7.62, new TreeMap<Double, Double>());
            table.get(4.0).get(7.62).put(11.176, 2.6);
            table.get(4.0).get(7.62).put(13.4112, 2.8);
            table.get(4.0).get(7.62).put(15.6464, 3.0);
            table.get(4.0).get(7.62).put(17.8816, 3.1);
            table.get(4.0).get(7.62).put(20.1168, 3.2);

            table.get(4.0).put(10.668, new TreeMap<Double, Double>());
            table.get(4.0).get(10.668).put(11.176, 2.3);
            table.get(4.0).get(10.668).put(13.4112, 2.6);
            table.get(4.0).get(10.668).put(15.6464, 2.8);
            table.get(4.0).get(10.668).put(17.8816, 2.9);
            table.get(4.0).get(10.668).put(20.1168, 3.1);

            table.get(4.0).put(13.716, new TreeMap<Double, Double>());
            table.get(4.0).get(13.716).put(11.176, 2.0);
            table.get(4.0).get(13.716).put(13.4112, 2.3);
            table.get(4.0).get(13.716).put(15.6464, 2.6);
            table.get(4.0).get(13.716).put(17.8816, 2.7);
            table.get(4.0).get(13.716).put(20.1168, 2.9);

            table.get(4.0).put(16.764, new TreeMap<Double, Double>());
            table.get(4.0).get(16.764).put(11.176, 1.7);
            table.get(4.0).get(16.764).put(13.4112, 2.1);
            table.get(4.0).get(16.764).put(15.6464, 2.3);
            table.get(4.0).get(16.764).put(17.8816, 2.6);
            table.get(4.0).get(16.764).put(20.1168, 2.7);

            table.get(4.0).put(19.812, new TreeMap<Double, Double>());
            table.get(4.0).get(19.812).put(11.176, 1.4);
            table.get(4.0).get(19.812).put(13.4112, 1.8);
            table.get(4.0).get(19.812).put(15.6464, 2.1);
            table.get(4.0).get(19.812).put(17.8816, 2.4);
            table.get(4.0).get(19.812).put(20.1168, 2.5);

            table.get(4.0).put(22.86, new TreeMap<Double, Double>());
            table.get(4.0).get(22.86).put(11.176, 1.1);
            table.get(4.0).get(22.86).put(13.4112, 1.6);
            table.get(4.0).get(22.86).put(15.6464, 1.9);
            table.get(4.0).get(22.86).put(17.8816, 2.2);
            table.get(4.0).get(22.86).put(20.1168, 2.4);
        }
    }
}
