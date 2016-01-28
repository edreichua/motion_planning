/**
 * This algorithm is based on the following paper:
 * Lydia E. Kavraki, Petr Svestka, Jean-Claude Latombe, and Mark H. Overmars,
 * Probabilistic roadmaps for path planning in high-dimensional configuration spaces, 
 * IEEE Transactions on Robotics and Automation 12 (4): 566–580.
 * http://dx.doi.org/10.1109/70.508439
 */
package assignment_motion_planning;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NavigableSet;
import java.util.Set;
import java.util.TreeSet;

public class PRMPlanner extends MotionPlanner {
    private int numberOfAttempts = 10;
    private Map<Vector, Map<Vector, Double>> roadMap;
    private int size;
    private boolean optimize = true;

    /**
     * Constructor
     * @param environment  the workspace
     * @param robot        a robot with a steering method
     */
    /**
     * Constructor
     * @param environment  the workspace
     * @param robot        a robot with a steering method
     */
    public PRMPlanner(Environment environment, Robot robot) {
        super(environment, robot);
        setup();
    }

    @Override
    public int getSize() {
        // YOU WILL WRITE THIS METHOD
        return size;
    }

    @Override
    protected void setup() {
        // YOU WILL WRITE THIS METHOD
        size = 0;
        roadMap = new HashMap<>();
    }

    @Override
    protected void growMap(int K) {
        // YOU WILL WRITE THIS METHOD
        for (int i = 0; i < K; i++) {
            Vector config = generateFreeConfiguration();
            addVertex(config);
            size++;
        }
    }

    /**
     * Add a free configuration to the roadmap
     * @param free  a free configuration
     */
    @SuppressWarnings("boxing")
    private void addVertex(Vector free) {
        // YOU WILL WRITE THIS METHOD
        List<Vector> neighbours = nearestKNeighbors(roadMap.keySet(),free,kValue());
        roadMap.put(free, new HashMap<>());

        for(Vector v: neighbours) {
            if ( optimize || (getEnvironment().isSteerable(getRobot(), free, v, RESOLUTION) &&
                    getEnvironment().isSteerable(getRobot(), v, free, RESOLUTION))) {
                double value = getRobot().getMetric(free, v);
                roadMap.get(free).put(v,value);
                roadMap.get(v).put(free, value);
                size++;
            }
        }
    }


    @Override
    protected void reset() {
        // YOU WILL WRITE THIS METHOD
        setup();
    }
    /**
     * 
     */
    @Override
    protected Trajectory query() {
        addVertex(getStart());
        addVertex(getGoal());
        return findPath();
    }
    
    /**
     * Generate a free configuration
     * @return a free configuration if possible, and null otherwise
     */
    private Vector generateFreeConfiguration() {
        // YOU WILL WRITE THIS METHOD
        for(int i=0; i<numberOfAttempts; i++){
            Vector config = getRobot().getRandomConfiguration(getEnvironment(),random);
            if(getEnvironment().isValidConfiguration(getRobot(), config)){
                return config;
            }
        }
        return null;
    }
    
    /**
     * The number of vertices for a new vertex attempts to connect.
     * The following paper describes a way to determine the value in order
     * to ensure asymptotic optimality.
     * Sertac Karaman and Emilio Frazzoli, 
     * Sampling-based Algorithms for Optimal Motion Planning, 
     * International Journal of Robotics Research, vol. 30, no.7, pp. 846-894, 2011.
     * http://dx.doi.org/10.1177/0278364911406761
     * @return number of neighbors for a new vertex attempts to connect.
     */
    private int kValue() {
        return 15; // Magic number suggested in Steven M. LaValle's "Planning Algorithms"
    }
    
    /**
     * Determine whether this edge connecting two configurations can be ignored.
     * The following paper describes a way to ignore some edges while maintaining
     * asymptotic optimality.
     * Weifu Wang, Devin J. Balkcom, and Amit Chakrabarti
     * A fast online spanner for roadmap construction
     * International Journal of Robotics Research, vol. 34, no.11, pp. 1418-1432, 2015.
     * http://dx.doi.org/10.1177/0278364915576491
     * @param u first configuration
     * @param v second configuration
     * @return true if this edge can be ignored, and false otherwise. 
     */
    private boolean safeToIgnore(Vector u, Vector v) {
        return false; // Extra credit!!
    }
    
    @Override
    protected Trajectory findPath() {
        List<Vector> path;
        if(!roadMap.containsKey(getStart()))
            addVertex(getStart());
        if(!roadMap.containsKey(getGoal()))
            addVertex(getGoal());

        if(optimize){
            do {
                path = aStar(getStart(), getGoal());
            }while(path!=null && !checkPath(path));

        }else{
            path = aStar(getStart(), getGoal());
        }
        return path != null ? convertToTrajectory(path) : null;
    }

    protected boolean checkPath(List<Vector> path){
        for(int i=path.lastIndexOf(getStart()); i<path.lastIndexOf(getGoal()); i++){
            if (!getEnvironment().isSteerable(getRobot(), path.get(i), path.get(i+1), RESOLUTION) ||
                    !getEnvironment().isSteerable(getRobot(), path.get(i+1), path.get(i), RESOLUTION)){

                Map<Vector,Double> childmap = roadMap.remove(path.get(i));
                childmap.remove(path.get(i + 1));
                roadMap.put(path.get(i), childmap);

                Map<Vector,Double> childmap2 = roadMap.remove(path.get(i+1));
                childmap2.remove(path.get(i));
                roadMap.put(path.get(i+1), childmap2);

                return false;
            }
        }
        return true;
    }

    
    /**
     * Convert a list of configurations to a corresponding trajectory based on the steering method
     * @param path a list of configurations
     * @return a trajectory
     */
    private Trajectory convertToTrajectory(List<Vector> path) {
        Trajectory result = new Trajectory();
        Vector previous = path.get(0);
        for (int i = 1; i < path.size(); ++i) {
            Vector next = path.get(i);
            result.append(getRobot().steer(previous, next));
            previous = next;
        }
        return result;
    }
    
    /**
     * Astar search
     * @return a path
     */
    @SuppressWarnings("boxing")
    private List<Vector> aStar(Vector start, Vector goal) {
        NavigableSet<Node> pq = new TreeSet<>();
        Map<Vector, Node> map = new HashMap<>();
        Node root = new Node(start, null, 0, getRobot().getMetric(start, goal));
        pq.add(root);
        map.put(start, root);
        while (!pq.isEmpty()) {
            Node node = pq.pollFirst();
            Vector configuration = node.getConfiguration();
            double cost = node.getCost();
            if (goal.equals(configuration))
                return backChain(node);
            for (Vector config : getSuccessors(configuration)){
                double heuristic = getRobot().getMetric(config, goal);
                double newCost = cost + roadMap.get(configuration).get(config); // YOU NEED TO MODIFY THIS LINE
                Node test = map.get(config);
                
                if (test != null) {
                    if (test.getPriority() > newCost + heuristic)
                        pq.remove(test);
                    else
                        continue;
                }
                Node newNode = new Node(config, node, newCost, heuristic);
                map.put(config, newNode);
                pq.add(newNode);
            }
        }
        return null;
    }
    
    /**
     * Get successors for a configuration
     * @param configuration
     * @return a collection of successors
     */
    private Collection<Vector> getSuccessors(Vector configuration) {
        // YOU WILL WRITE THIS METHOD
        if(roadMap.containsKey(configuration))
            return roadMap.get(configuration).keySet();
        else
            return null;
    }
    
    /**
     * Backchain to construct a path
     * @param node the end node
     * @return a path
     */
    private static List<Vector> backChain(Node node) {
        LinkedList<Vector> result = new LinkedList<>();
        for (Node current = node; current != null; current = current.getParent()) {
            result.addFirst(current.getConfiguration());
        }
        return result;
    }
    
    final class Node implements Comparable<Node> {
        private Vector configuration;
        private Node parent;
        private double heuristic;
        private double cost;
        
        public Node(Vector config, Node p, double cost, double heuristic) {
            this.configuration = config;
            this.parent = p;
            this.heuristic = heuristic;
            this.cost = cost;
        }
        
        @Override
        public int compareTo(Node o) {
            int comparison = Double.compare(getPriority(), o.getPriority());
            return comparison == 0 ? configuration.compareTo(o.getConfiguration()) : comparison;
        }
        
        /**
         * Get the parent
         * @return the parent
         */
        public Node getParent(){
            return parent;
        }
        
        /**
         * Get the configuration
         * @return the configuration
         */
        public Vector getConfiguration() {
            return configuration;
        }
        
        /**
         * Get the cost
         * @return the cost
         */
        public double getCost() {
            return cost;
        }
        
        /**
         * Get the heuristic
         * @return the heuristic
         */
        public double getHeuristic() {
            return heuristic;
        }
        
        /**
         * Get the priority 
         * @return the priority
         */
        public double getPriority(){
            return getCost() + getHeuristic();
        }
    }
}
