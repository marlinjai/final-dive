package s0582535;

import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;

import java.awt.*;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.*;
import java.util.List;

public class MyDiverAi extends lenz.htw.ai4g.ai.AI {
    private Point2D.Float diverPos;
    private Stack<Point2D> pathToSwim;
    private float diverAngle;
    private static final float MAX_ACCELERATION = 1.0f;

    ArrayList<Point2D> pearlsList = new ArrayList<>();

    double rayLength = 43.0; // Length of the rays
    double angleLeft;
    double angleRight;

    int numPoints = 10; // Number of points along each ray
    Point2D.Float[] pointsLeft;
    Point2D.Float[] pointsRight;

    private final int stepSize = 35; // Grid size
    private Map<Point2D, List<Point2D>> graph;
    private Set<Point2D> collectedPearls;

    public MyDiverAi(Info info) {
        super(info);
        this.enlistForTournament(582535, 585089);
        diverPos = new Point2D.Float(info.getX(), info.getY());
        pathToSwim = new Stack<>();
        diverAngle = info.getOrientation();
        pointsLeft = new Point2D.Float[numPoints];
        pointsRight = new Point2D.Float[numPoints];
        collectedPearls = new HashSet<>();

        graph = new HashMap<>();
        precomputeGraph(info.getScene().getWidth(), info.getScene().getHeight(), Arrays.asList(info.getScene().getObstacles()), info.getScene().getPearl());
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

    private void precomputeGraph(int width, int height, List<Path2D> obstacles, Point[] pearls) {
        for (int x = 0; x < width; x += stepSize) {
            for (int y = 0; y < height; y += stepSize) {
                Point2D current = new Point2D.Float(x, y);
                if (!isInObstacle(current, obstacles)) {
                    graph.put(current, getNeighbors(current, obstacles));
                }
            }
        }

        // Add pearls to the graph
        for (Point pearl : pearls) {
            Point2D pearlPos = new Point2D.Float(pearl.x, pearl.y);
            pearlsList.add(pearlPos);
            if (!isInObstacle(pearlPos, obstacles)) {
                graph.put(pearlPos, getNeighbors(pearlPos, obstacles));
            }
        }
    }

    private boolean isInObstacle(Point2D point, List<Path2D> obstacles) {
        for (Path2D obstacle : obstacles) {
            if (obstacle.contains(point)) {
                return true;
            }
        }
        return false;
    }

    private List<Point2D> getNeighbors(Point2D point, List<Path2D> obstacles) {
        List<Point2D> neighbors = new ArrayList<>();

        for (double angle = 0; angle < 360; angle += 90) {
            double rad = Math.toRadians(angle);
            Point2D neighbor = new Point2D.Double(point.getX() + Math.cos(rad) * stepSize, point.getY() + Math.sin(rad) * stepSize);

            if (!isInObstacle(neighbor, obstacles)) {
                neighbors.add(neighbor);
            }
        }
        return neighbors;
    }

    private Map<Point2D, Point2D> dijkstra(Point2D start, List<Point2D> goals) {
        Map<Point2D, Double> dist = new HashMap<>();
        Map<Point2D, Point2D> prev = new HashMap<>();
        PriorityQueue<Point2D> pq = new PriorityQueue<>(Comparator.comparingDouble(dist::get));
        Set<Point2D> visited = new HashSet<>();

        dist.put(start, 0.0);
        pq.add(start);

        while (!pq.isEmpty()) {
            Point2D current = pq.poll();

            if (visited.contains(current)) continue;
            visited.add(current);

            for (Point2D neighbor : graph.getOrDefault(current, new ArrayList<>())) {
                double altDist = dist.get(current) + current.distance(neighbor);

                if (altDist < dist.getOrDefault(neighbor, Double.MAX_VALUE)) {
                    dist.put(neighbor, altDist);
                    prev.put(neighbor, current);
                    pq.add(neighbor);
                }
            }
        }

        // Find the closest goal
        Point2D closestGoal = null;
        double shortestDist = Double.MAX_VALUE;
        for (Point2D goal : goals) {
            double goalDist = dist.getOrDefault(goal, Double.MAX_VALUE);
            if (goalDist < shortestDist) {
                closestGoal = goal;
                shortestDist = goalDist;
            }
        }

        // Reconstruct the shortest path to the closest goal
        Map<Point2D, Point2D> path = new HashMap<>();
        for (Point2D at = closestGoal; at != null; at = prev.get(at)) {
            path.put(at, prev.get(at));
        }
        return path;
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
        // Draw current path to swim (all points in the stack)
        if (!pathToSwim.isEmpty()) {
            gfx.setColor(Color.RED);
            Point2D prevPoint = null;
            for (Point2D point : pathToSwim) {
                if (prevPoint != null) {
                    gfx.drawLine((int) prevPoint.getX(), (int) prevPoint.getY(), (int) point.getX(), (int) point.getY());
                }
                gfx.fillOval((int) point.getX() - 2, (int) point.getY() - 2, 4, 4); // Draw points as small circles
                prevPoint = point;
            }
        }

        // Draw diver's current target (top of the stack)
        if (!pathToSwim.isEmpty()) {
            Point2D top = pathToSwim.peek();
            gfx.setColor(Color.GREEN);
            gfx.drawOval((int) top.getX() - 5, (int) top.getY() - 5, 10, 10);
        }

        // Draw the rays for obstacle detection
        if (pointsLeft != null && pointsLeft[0] != null) {
            gfx.setColor(Color.BLUE);
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) pointsLeft[pointsLeft.length - 1].getX(), (int) pointsLeft[pointsLeft.length - 1].getY());
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) pointsRight[pointsRight.length - 1].getX(), (int) pointsRight[pointsRight.length - 1].getY());
        }

        // Draw all keys of the graph
        gfx.setColor(Color.BLACK);
        for (Point2D point : graph.keySet()) {
            gfx.drawOval((int) point.getX() - 2, (int) point.getY() - 2, 4, 4);
        }
    }

    @Override
    public PlayerAction update() {
        diverPos.setLocation(info.getX(), info.getY());
        diverAngle = info.getOrientation();

        angleLeft = diverAngle - Math.toRadians(45); // 45 degrees to the left
        angleRight = diverAngle + Math.toRadians(45); // 45 degrees to the right

        // Initialize points arrays if they are null
        if (pointsLeft == null) {
            pointsLeft = new Point2D.Float[numPoints];
        }
        if (pointsRight == null) {
            pointsRight = new Point2D.Float[numPoints];
        }

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
        if (pathToSwim.isEmpty()) {
            if (pearlsList.isEmpty()) {
                Point[] pearls = info.getScene().getPearl();
                for (Point pearl : pearls) {
                    Point2D pearlPos = new Point2D.Float(pearl.x, pearl.y);
                    if (!collectedPearls.contains(pearlPos)) {
                        pearlsList.add(pearlPos);
                    }
                }
            }
            // Use Dijkstra to find the shortest path to the closest pearl
            if (!pearlsList.isEmpty()) {
                Map<Point2D, Point2D> shortestPath = dijkstra(diverPos, pearlsList);
                Point2D closestPearl = null;
                double shortestDist = Double.MAX_VALUE;
                for (Point2D pearl : pearlsList) {
                    double dist = diverPos.distance(pearl);
                    if (dist < shortestDist) {
                        closestPearl = pearl;
                        shortestDist = dist;
                    }
                }
                // Here is the missing part
                if (closestPearl != null) {
                    for (Point2D at = closestPearl; at != null; at = shortestPath.get(at)) {
                        pathToSwim.push(at);
                    }
                }
            }
        }

        // Check if the diver has reached the current target pearl
        if (!pathToSwim.isEmpty() && pathToSwim.peek().distance(diverPos) < stepSize) {
            Point2D reachedPearl = pathToSwim.pop();
            collectedPearls.add(reachedPearl);
            pearlsList.remove(reachedPearl);
        }



        Point2D.Float targetPearl = pathToSwim.isEmpty() ? null : (Point2D.Float) pathToSwim.peek();
        float alignAcc = targetPearl == null ? 0 : align(targetPearl, obstacleLeft, obstacleRight, turnFactorLeft, turnFactorRight);

        return new DivingAction(info.getMaxAcceleration(), alignAcc);
    }

    public Boolean insideCircle(Point2D point, Point2D center, double radius) {
        return point.distanceSq(center) <= radius * radius;
    }
}
