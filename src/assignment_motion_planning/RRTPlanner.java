/**
 * This algorithm is from the following paper
 * Steven M. LaValle and James  Kuffner Jr.,
 * Randomized Kinodynamic Planning, 
 * The International Journal of Robotics Research 20 (5), pp. 378–400.
 * http://dx.doi.org/10.1177/02783640122067453
 */

package assignment_motion_planning;

import java.util.*;

import javafx.geometry.*;
import javafx.util.Pair;
import sun.awt.geom.Curve;

public class RRTPlanner extends MotionPlanner {
    private static final double DEFAULT_DELTA =0.1;  // Duration for the control
    private static final double EXTRA_DELTA = 0.5; // Duration for the control
    private Map<Vector, Edge> parents;
    private int size;
    private boolean optimize = true;
    private Random rand = new Random();

    /**
     * Constructor 
     * @param environment the workspace
     * @param robot       the robot
     */
    public RRTPlanner(Environment environment, Robot robot) {
        super(environment, robot);
    }
    
    @Override
    public List<Pair<Point2D, Point2D>> getEdges() {
        // YOU WILL WRITE THIS METHOD
        List<Pair<Point2D,Point2D>> result = new ArrayList<>();
        for(Map.Entry e: parents.entrySet()){
            Edge temp = (Edge) e.getValue();
            if(temp != null)
                result.add(temp.getPair());
        }

        return result;
    }
    
    @Override
    public int getSize() {
        // YOU WILL WRITE THIS METHOD
        return size;
    }

    @Override
    protected void setup() {
        // YOU WILL WRITE THIS METHOD
        parents = new HashMap<>();
        parents.put(getStart(),null);
        size = 0;
    }
    
    @Override
    protected void growMap(int K) {
        // YOU WILL WRITE THIS METHOD
        int i = 0;
        double value;
        while( i < K ){

            Vector qrand = getRobot().getRandomConfiguration(getEnvironment(), random);

            if(optimize && rand.nextInt(100)<=1) {
                qrand = new Vector(getGoal().get(0), getGoal().get(1), getGoal().get(2));
            }

            Vector qnear = nearestNeighbor(parents.keySet(), qrand);

            if(optimize) {
                value = rand.nextDouble() * EXTRA_DELTA;
            }else {
                value = DEFAULT_DELTA;
            }

            if(newConf(qnear, value))
                i++;

        }
        size += K;
    }

    /**
     * Generate a new configuration from a configuration and insert it
     * @param qnear    the beginning configuration of the random motion
     * @param duration the duration of the random motion
     * @return true if one new configuration is inserted, and false otherwise
     */
    @SuppressWarnings("boxing")
    private boolean newConf(Vector qnear, double duration) {
        // YOU WILL WRITE THIS METHOD
        Vector control = getRobot().getRandomControl(random);
        Vector end = getRobot().move(qnear,control,duration);

        if(getEnvironment().isValidMotion(getRobot(),qnear,new Trajectory(control,duration),RESOLUTION)) {

            if(!parents.containsKey(end)){
                parents.put(end, new Edge(qnear, control, duration));
                return true;
            }

        }
        return false;
    }
    
    @SuppressWarnings("boxing")
    @Override
    protected Trajectory findPath() {
        // YOU WILL WRITE THIS METHOD
        Vector goal = nearestNeighbor(parents.keySet(), getGoal());
        Trajectory result = new Trajectory();
        Vector node = goal;
        while(true){
            Edge e = parents.get(node);
            Trajectory temp = new Trajectory(e.getControl(),e.getDuration());
            temp.append(result);
            result = temp;
            node = e.getConfiguration();
            if(node.equals(getStart()))
                return result;
        }

    }






    @Override
    protected void reset() {
        // YOU WILL WRITE THIS METHOD
        setup();
    }

    final class Edge{
        private Vector configuration;
        private Vector control;
        private double duration;
        private Pair<Point2D,Point2D> pair;

        public Edge(Vector configuration, Vector control, double duration){
            this.configuration = configuration;
            this.control = control;
            this.duration = duration;
            Vector v = getRobot().move(configuration,control,duration);
            Point2D p1 = new Point2D(configuration.get(0),configuration.get(1));
            Point2D p2 = new Point2D(v.get(0),v.get(1));
            this.pair = new Pair<>(p1,p2);
        }

        public Vector getConfiguration() {
            return configuration;
        }

        public Vector getControl(){
            return control;
        }

        public double getDuration(){
            return duration;
        }

        public Pair<Point2D,Point2D> getPair() {
            return pair;
        }

    }

}
