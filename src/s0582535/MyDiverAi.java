package s0582535;

import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;

import java.awt.*;
import java.awt.geom.*;
import java.util.*;
import java.util.List;

class Node {
    Point2D point;
    List<Edge> edges;

    public Node(Point2D point) {
        this.point = point;
        this.edges = new ArrayList<>();
    }
}

class Edge {
    Node target;
    double weight;

    public Edge(Node target, double weight) {
        this.target = target;
        this.weight = weight;
    }
}

class Graph {
    Map<Point2D, Node> nodes;

    public Graph() {
        this.nodes = new HashMap<>();
    }

    public void addNode(Point2D point) {
        nodes.put(point, new Node(point));
    }

    public void addEdge(Point2D from, Point2D to, double weight) {
        Node fromNode = nodes.get(from);
        Node toNode = nodes.get(to);
        if (fromNode != null && toNode != null) {
            fromNode.edges.add(new Edge(toNode, weight));
        }
    }

    public Node getNode(Point2D point) {
        return nodes.get(point);
    }
}
class Dijkstra {
    public static List<Point2D> findShortestPath(Graph graph, Point2D start, Point2D goal) {
        Map<Point2D, Double> distances = new HashMap<>();
        Map<Point2D, Point2D> previous = new HashMap<>();
        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingDouble(node -> distances.getOrDefault(node.point, Double.MAX_VALUE)));


        for (Point2D point : graph.nodes.keySet()) {
            distances.put(point, Double.MAX_VALUE);
            previous.put(point, null);
        }
        distances.put(start, 0.0);
        Node startNode = graph.getNode(start);
        if (startNode != null) {
            queue.add(startNode);
        }

                while (!queue.isEmpty()) {
                    Node current = queue.poll();
                   // System.out.println("Current node: " + current.point); // Debug print
                    if (current.point.equals(goal)) break;

                    for (Edge edge : current.edges) {
                        Node neighbor = edge.target;
                        double newDist = distances.get(current.point) + edge.weight;
                        if (newDist < distances.get(neighbor.point) && neighbor != null) {
                            distances.put(neighbor.point, newDist);
                            previous.put(neighbor.point, current.point);
                            queue.add(neighbor);
                        }
                    }
                }

                if (previous.get(goal) == null) {
                    System.out.println("Goal is not reachable from the start"); // Debug print
                    return Collections.emptyList(); // Goal is not reachable from the start
                }

                List<Point2D> path = new ArrayList<>();
                for (Point2D at = goal; at != null; at = previous.get(at)) {
                    path.add(at);
                }
                Collections.reverse(path);
   //             System.out.println("Shortest path: " + path); // Debug print
                return path;
            }
        }

public class MyDiverAi extends lenz.htw.ai4g.ai.AI {
    private Point2D.Float diverPos;

    private Point2D.Float initDiverPos;
    private Stack<Point2D> pathToSwim;
    private List<Point2D> pearlsToCollect;

    private List<Point2D> airLine;
    private float diverAngle;
    private static final float MAX_ACCELERATION = 1.0f;

    double rayLength = 27.0; // Length of the rays
    double angleLeft;
    double angleRight;

    // Calculate points along the rays
    int numPoints = 10; // Number of points along each ray
    Point2D.Float[] pointsLeft;
    Point2D.Float[] pointsRight;

    Point2D lastVisitedNode;

    private float air;

    private Graph graph;
    private float stepSize = 40;

    private static final double MIN_DISTANCE_TO_OBSTACLE = 25.0;

    public MyDiverAi(Info info) {
        super(info);
        this.enlistForTournament(582535, 585089);
        diverPos = new Point2D.Float(info.getX(), info.getY());
        initDiverPos = new Point2D.Float(info.getX(), info.getY());
        pathToSwim = new Stack<>();
        pearlsToCollect = new ArrayList<>();
        diverAngle = info.getOrientation();
        graph = new Graph();
        pointsLeft = new Point2D.Float[numPoints];
        pointsRight = new Point2D.Float[numPoints];
        lastVisitedNode = null;
        air = info.getAir();
        airLine = new ArrayList<>();
        createGraph();
        initializePearls();
    }

    private void createGraph() {
        // Add pearls as nodes
        Point[] pearls = info.getScene().getPearl();
        for (Point pearl : pearls) {
            Point2D pearlPoint = new Point2D.Float(pearl.x, pearl.y);
            graph.addNode(pearlPoint);
        }

        // Add air refill points as nodes
        for (float x = 0; x < info.getScene().getWidth(); x += stepSize) {
            Point2D airPoint = new Point2D.Float(x, 0);
            graph.addNode(airPoint);
            airLine.add(airPoint);
        }

        // Add grid points below the airline (y > 40)
        float sceneWidth = info.getScene().getWidth();
        float sceneHeight = info.getScene().getHeight();
        Path2D[] obstacles = info.getScene().getObstacles();

        graph.addNode(initDiverPos);

        for (float x = 0; x < sceneWidth; x += stepSize) {
            for (float y = stepSize; y < sceneHeight; y += stepSize) {
                Point2D gridPoint = new Point2D.Float(x, y);
                boolean insideObstacle = false;
                boolean tooCloseToObstacle = false;

                for (Path2D obstacle : obstacles) {
                    if (obstacle.contains(gridPoint.getX(), gridPoint.getY())) {
                        insideObstacle = true;
                        break;
                    }

                    if (getDistanceToObstacle(gridPoint, obstacle) < MIN_DISTANCE_TO_OBSTACLE) {
                        tooCloseToObstacle = true;
                        break;
                    }
                }

                if (!insideObstacle && !tooCloseToObstacle) {
                    graph.addNode(gridPoint);
                }
            }
        }

        // Add edges between nodes
        addEdges();
    }

    private double getDistanceToObstacle(Point2D point, Path2D obstacle) {
        double[] coords = new double[6];
        double minDistance = Double.MAX_VALUE;

        for (PathIterator pi = obstacle.getPathIterator(null); !pi.isDone(); pi.next()) {
            int type = pi.currentSegment(coords);
            Point2D obstaclePoint;

            if (type == PathIterator.SEG_MOVETO || type == PathIterator.SEG_LINETO) {
                obstaclePoint = new Point2D.Double(coords[0], coords[1]);
                double distance = point.distance(obstaclePoint);
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
        }

        return minDistance;
    }

    private boolean isObstructed(Point2D from, Point2D to) {
        Path2D[] obstacles = info.getScene().getObstacles();
        Line2D line = new Line2D.Double(from, to);

        for (Path2D obstacle : obstacles) {
            PathIterator pi = obstacle.getPathIterator(null);
            double[] coords = new double[6];
            double[] lastCoords = new double[2];
            double[] firstCoords = new double[2];

            if (!pi.isDone()) {
                pi.currentSegment(firstCoords);
                lastCoords[0] = firstCoords[0];
                lastCoords[1] = firstCoords[1];
                pi.next();
            }

            while (!pi.isDone()) {
                int type = pi.currentSegment(coords);

                if (type == PathIterator.SEG_LINETO) {
                    Line2D segment = new Line2D.Double(lastCoords[0], lastCoords[1], coords[0], coords[1]);
                    if (line.intersectsLine(segment)) {
                        return true;
                    }
                    lastCoords[0] = coords[0];
                    lastCoords[1] = coords[1];
                } else if (type == PathIterator.SEG_CLOSE) {
                    Line2D segment = new Line2D.Double(lastCoords[0], lastCoords[1], firstCoords[0], firstCoords[1]);
                    if (line.intersectsLine(segment)) {
                        return true;
                    }
                }

                pi.next();
            }
        }

        return false;
    }

    private void addEdges() {
        for (Node fromNode : graph.nodes.values()) {
            for (Node toNode : graph.nodes.values()) {
                if (!fromNode.equals(toNode) && !isObstructed(fromNode.point, toNode.point)) {
                    double distance = fromNode.point.distance(toNode.point);
                    graph.addEdge(fromNode.point, toNode.point, distance);
                }
            }
        }
    }

    private void initializePearls() {
        Point[] pearls = info.getScene().getPearl();
        for (Point pearl : pearls) {
            pearlsToCollect.add(new Point2D.Float(pearl.x, pearl.y));
        }
        pearlsToCollect.sort(Comparator.comparingDouble(Point2D::getX));
    }

    private void recomputePath(Graph graph, Point2D start, Point2D target) {
        pathToSwim.clear();
        if (!pearlsToCollect.isEmpty()) {
            List<Point2D> path = Dijkstra.findShortestPath(graph, start, target);
            System.out.println("DIk res: " + path);
            Collections.reverse(path);
            pathToSwim.addAll(path);
        }
    }

    private boolean isInReach(Point2D start, Point2D target) {

        List<Point2D> pathToTarget = Dijkstra.findShortestPath(graph, start, target);
        List<Point2D> totalPath = new ArrayList<>();
        for (int i = 0; i < pearlsToCollect.size(); i++) {
            if(i == 0 ) {
                totalPath.addAll(Dijkstra.findShortestPath(graph, start, pearlsToCollect.get(i)));
            }
            else {
                totalPath.addAll(Dijkstra.findShortestPath(graph, pearlsToCollect.get(i-1), pearlsToCollect.get(i)));
            }
        }


        Point2D closestAirPoint = findClosestAirPoint(target);
        List<Point2D> pathToAir = (closestAirPoint != null) ? Dijkstra.findShortestPath(graph, target, closestAirPoint) : Collections.emptyList();

        // Calculate the total path by combining path to target and path to air
        List<Point2D> fullPath = new ArrayList<>(pathToTarget);
        fullPath.addAll(pathToAir);

        // Calculate the total distance of the path
        double totalPathDistance = 0;
        double midPathDistance = 0;
        Point2D previousPoint = start;
        for (Point2D point : totalPath) {
            totalPathDistance += previousPoint.distance(point);
            previousPoint = point;
        }
        previousPoint = start;
        for (Point2D point : fullPath) {
            midPathDistance += previousPoint.distance(point);
            previousPoint = point;
        }

        // Assuming air consumption rate is 1 and checking if the diver can reach the end with available air
        double airConsumptionRate = 0.9085;
        double totalRequiredAir = totalPathDistance * airConsumptionRate;
        double requiredAir = midPathDistance * airConsumptionRate;

        boolean res = (requiredAir <= air*0.95 || totalRequiredAir <= air*0.95);

        System.out.println("result of is In reach: "+res);
        return res; // air should be the current air level of the diver
    }


    private Point2D findClosestAirPoint(Point2D target) {
        if (airLine.isEmpty()) {
            return null;  // Handle case where no air points are available
        }

        Point2D closestPoint = null;
        double minDistance = Double.MAX_VALUE;  // Start with the maximum possible value

        for (Point2D p : airLine) {
            double currentDistance = p.distance(target);  // Calculate distance to the target point
            if (currentDistance < minDistance) {
                minDistance = currentDistance;  // Update minimum distance
                closestPoint = p;  // Update closest point
            }
        }

        return closestPoint;  // Return the closest air point found
    }

    private Point2D findClosestPearl() {

        Point2D closestPearl = null;
        double minDistance = Double.MAX_VALUE;  // Start with the maximum possible value

        for (Point2D pearl : pearlsToCollect) {
            double dist = pearl.distance(diverPos);
            if (dist < minDistance) {
                minDistance = dist;
                closestPearl = pearl;
            }
        }
        return closestPearl;
    }


    @Override
    public String getName() {
        return "Lion";
    }

    @Override
    public Color getPrimaryColor() {
        return Color.DARK_GRAY;
    }

    @Override
    public Color getSecondaryColor() {
        return Color.green;
    }

    public Boolean insideCircle(Point2D point, Point2D center, double radius) {
        return point.distanceSq(center) <= radius * radius;
    }

    private float align(Point2D goal, boolean obstacleLeft, boolean obstacleRight, float turnFactorLeft, float turnFactorRight) {
        float targetOrientation = (float) -Math.atan2(goal.getY() - diverPos.getY(), goal.getX() - diverPos.getX());
        float angularDifference = targetOrientation - diverAngle;

        // If there's an obstacle on the left, adjust the target orientation to the right
        if (obstacleLeft) {
            targetOrientation += Math.toRadians(45) * turnFactorLeft;
        }

        // If there's an obstacle on the right, adjust the target orientation to the left
        if (obstacleRight) {
            targetOrientation -= Math.toRadians(45) * turnFactorRight;
        }

        angularDifference = targetOrientation - diverAngle;

        if (Math.abs(angularDifference) > 0.001f) {
            float result;
            if (Math.abs(angularDifference) < 0.2f) {
                result = angularDifference * info.getMaxAbsoluteAngularVelocity() / 0.2f;
            } else {
                result = Math.signum(angularDifference) * info.getMaxAbsoluteAngularVelocity();
            }

            return result - info.getAngularVelocity();
        }
        return 0;
    }

    @Override
    public void drawDebugStuff(Graphics2D gfx) {
        if (!pathToSwim.isEmpty()) {
            Point2D top = pathToSwim.peek();
            gfx.setColor(Color.YELLOW);
            gfx.drawOval((int) top.getX() - 5, (int) top.getY() - 5, 10, 10);
        }
        if (pointsLeft != null && pointsLeft[numPoints - 1] != null && pointsRight != null && pointsRight[numPoints - 1] != null) {
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) pointsLeft[numPoints - 1].getX(), (int) pointsLeft[numPoints - 1].getY());
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) pointsRight[numPoints - 1].getX(), (int) pointsRight[numPoints - 1].getY());
        }


        gfx.setColor(Color.BLUE);
        for (Point2D s : pathToSwim) {
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) s.getX(), (int) s.getY());
        }


// Draw the pearls
        gfx.setColor(Color.RED);
        for (Point2D pearl : pearlsToCollect) {
            gfx.fillOval((int) pearl.getX() - 3, (int) pearl.getY() - 3, 6, 6);
        }

        // Draw the air refill points
        gfx.setColor(Color.CYAN);
        for (Point2D airPoint : airLine) {
            gfx.fillOval((int) airPoint.getX() - 3, (int) airPoint.getY() - 3, 6, 6);
        }

        // Draw the grid points
        gfx.setColor(Color.MAGENTA);
        for (Node node : graph.nodes.values()) {
            gfx.fillOval((int) node.point.getX() - 3, (int) node.point.getY() - 3, 6, 6);
        }

    /*    // Draw the graph nodes and edges
        gfx.setColor(Color.PINK);
        for (Node node : graph.nodes.values()) {
            // Draw node
            gfx.fillOval((int) node.point.getX() - 3, (int) node.point.getY() - 3, 6, 6);

            gfx.setColor(Color.green);
            // Draw edges
            for (Edge edge : node.edges) {
                gfx.drawLine((int) node.point.getX(), (int) node.point.getY(), (int) edge.target.point.getX(), (int) edge.target.point.getY());
            }
        }*/
    }

    @Override
    public PlayerAction update() {
        diverPos.setLocation(info.getX(), info.getY());
        diverAngle = info.getOrientation();
        air = info.getAir();


        angleLeft = diverAngle - Math.toRadians(45); // 45 degrees to the left
        angleRight = diverAngle + Math.toRadians(45); // 45 degrees to the right

        // Calculate points along the rays
        for (int i = 0; i < numPoints; i++) {
            double factor = (i + 1) / (double) numPoints;
            pointsLeft[i] = new Point2D.Float((float) (diverPos.getX() + Math.cos(angleLeft) * rayLength * factor), (float) (diverPos.getY() - Math.sin(angleLeft) * rayLength * factor));
            pointsRight[i] = new Point2D.Float((float) (diverPos.getX() + Math.cos(angleRight) * rayLength * factor), (float) (diverPos.getY() - Math.sin(angleRight) * rayLength * factor));
        }

        Path2D[] obstacles = info.getScene().getObstacles();

        // Initialize obstacle flags and distances
        boolean obstacleLeft = false;
        boolean obstacleRight = false;
        double minDistanceLeft = Double.MAX_VALUE;
        double minDistanceRight = Double.MAX_VALUE;

        // Check if these points are within any obstacle
        for (Path2D obstacle : obstacles) {
            for (int i = 0; i < numPoints; i++) {
                if (obstacle.contains(pointsLeft[i].x, pointsLeft[i].y)) {
                    obstacleLeft = true;
                    minDistanceLeft = Math.min(minDistanceLeft, pointsLeft[i].distance(diverPos));
                }
                if (obstacle.contains(pointsRight[i].x, pointsRight[i].y)) {
                    obstacleRight = true;
                    minDistanceRight = Math.min(minDistanceRight, pointsRight[i].distance(diverPos));
                }
            }
        }

        // Calculate turn factors based on distance to obstacles
        float turnFactorLeft = obstacleLeft ? (float) (15.0 / minDistanceLeft) : 1.0f;
        float turnFactorRight = obstacleRight ? (float) (15.0 / minDistanceRight) : 1.0f;

        // Populate the pathToSwim stack if it's empty
        //System.out.println(pearlsToCollect.size() );
        if (pathToSwim.isEmpty() && !pearlsToCollect.isEmpty()) {

            if (lastVisitedNode == null) recomputePath(graph, initDiverPos, findClosestPearl());
            else {

                Point2D closestPearl = findClosestPearl();
                if (isInReach(lastVisitedNode, closestPearl)) {
                    recomputePath(graph, lastVisitedNode, closestPearl);
                } else {
                    Point2D closestAirPoint = findClosestAirPoint(lastVisitedNode);

                    if (lastVisitedNode.equals(closestAirPoint)) {
                        if (closestPearl.getX() - lastVisitedNode.getX() < 0) {
                            Point2D airPoint = new Point2D.Float((float) (closestAirPoint.getX() - 3*stepSize), (float) closestAirPoint.getY());
                            recomputePath(graph, lastVisitedNode, airPoint);
                        } else if (closestPearl.getX() - lastVisitedNode.getX() > 0) {
                            Point2D airPoint = new Point2D.Float((float) (closestAirPoint.getX() + 3*stepSize), (float) closestAirPoint.getY());
                            recomputePath(graph, lastVisitedNode, airPoint);
                        }
                    } else if (closestAirPoint != lastVisitedNode) {
                        recomputePath(graph, lastVisitedNode, closestAirPoint);
                    }
                }
            }
        }


            if (!pathToSwim.isEmpty()) {

                    Point2D reachedPoint = pathToSwim.peek();


                if (insideCircle(diverPos, reachedPoint, 11.5)) {
                    if (pearlsToCollect.contains(reachedPoint)) {
                        pearlsToCollect.remove(reachedPoint);
                        System.out.println("Pearl collected");
                    }
                    if (insideCircle(diverPos, reachedPoint, 15) ) {
                    pathToSwim.pop();
                    }
                    lastVisitedNode = reachedPoint;


                }
            }

            Point2D.Float targetPearl = pathToSwim.isEmpty() ? null : (Point2D.Float) pathToSwim.peek();
        System.out.println("Target pearl: " + targetPearl);
            float alignAcc = targetPearl == null ? 0 : align(targetPearl, obstacleLeft, obstacleRight, turnFactorLeft, turnFactorRight);


            // Now you can use obstacleLeft and obstacleRight to determine if there's an obstacle in the direction of the rays

            for (int i = 0; i < obstacles.length; i++) {
                PathIterator pi = obstacles[i].getPathIterator(null);
                while (!pi.isDone()) {
                    float[] array = new float[6];
                    pi.currentSegment(array);
                    float x = array[0];
                    float y = array[1];
                    pi.next();
                }
            }


            return new DivingAction(info.getMaxAcceleration(), alignAcc);
        }

    }

