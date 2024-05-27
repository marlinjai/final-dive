package s0582535;

import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;

import java.awt.*;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Stack;

public class MyDiverAi extends lenz.htw.ai4g.ai.AI {
    private Point2D.Float diverPos;
    private Stack<Point2D> pathToSwim;
    private Stack<Point2D> pearlsToCollect;
    private float diverAngle;
    private static final float MAX_ACCELERATION = 1.0f;

    double rayLength = 43.0; // Length of the rays
    double angleLeft ;
    double angleRight ;

    // Calculate points along the rays
    int numPoints = 10; // Number of points along each ray
    Point2D.Float[] pointsLeft = new Point2D.Float[numPoints];
    Point2D.Float[] pointsRight = new Point2D.Float[numPoints];

    public MyDiverAi(Info info) {
        super(info);
        this.enlistForTournament(582535,585089);
        diverPos = new Point2D.Float(info.getX(), info.getY());
        pathToSwim = new Stack<>();
        pearlsToCollect = new Stack<>();
        diverAngle = info.getOrientation();
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

    public ArrayList<Point2D> sortPearls(ArrayList<Point2D> pearls) {
        Collections.sort(pearls, Comparator.comparingDouble(Point2D::getX));
        return pearls;
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
            result = angularDifference * info.getMaxAbsoluteAngularVelocity()/0.2f;
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
            gfx.drawOval((int) top.getX() - 5, (int) top.getY() - 5, 10, 10);
        }
        if (pointsLeft != null){
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) pointsLeft[pointsLeft.length-1].getX(), (int) pointsLeft[pointsLeft.length-1].getY());
            gfx.drawLine((int) diverPos.getX(), (int) diverPos.getY(), (int) pointsRight[pointsRight.length-1].getX(), (int) pointsRight[pointsRight.length-1].getY());
        }
    }

    @Override
public PlayerAction update() {
    diverPos.setLocation(info.getX(), info.getY());
    diverAngle = info.getOrientation();

        angleLeft = diverAngle - Math.toRadians(45); // 45 degrees to the left
        angleRight = diverAngle + Math.toRadians(45); // 45 degrees to the right


    // Calculate points along the rays
    for (int i = 0; i < numPoints; i++) {
        double factor = (i + 1) / (double) numPoints;
        pointsLeft[i] = new Point2D.Float((float)(diverPos.getX() + Math.cos(angleLeft) * rayLength * factor), (float)(diverPos.getY() - Math.sin(angleLeft) * rayLength * factor));
        pointsRight[i] = new Point2D.Float((float)(diverPos.getX() + Math.cos(angleRight) * rayLength * factor), (float)(diverPos.getY() - Math.sin(angleRight) * rayLength * factor));
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
        Point[] pearls = info.getScene().getPearl();
        ArrayList<Point2D> pearlsList = new ArrayList<>();
        for (Point pearl : pearls) {
            pearlsList.add(new Point2D.Float(pearl.x, pearl.y));
        }
        pearlsList = sortPearls(pearlsList);
        for (Point2D pearl : pearlsList) {
            pathToSwim.push(new Point2D.Float((float)pearl.getX(),(float) pearl.getY()));
        }

    }

    if (!pathToSwim.isEmpty() && insideCircle(diverPos, pathToSwim.peek(), 10)) {
        pathToSwim.pop();
        System.out.println("Pearl collected");
    }

    Point2D.Float targetPearl = pathToSwim.isEmpty() ? null : (Point2D.Float) pathToSwim.peek();
    //System.out.println("Target pearl: " + targetPearl);
    float alignAcc = targetPearl == null ? 0 : align(targetPearl, obstacleLeft, obstacleRight, turnFactorLeft, turnFactorRight);


    // Now you can use obstacleLeft and obstacleRight to determine if there's an obstacle in the direction of the rays


    PathIterator pi = obstacles[0].getPathIterator(null);
    while (!pi.isDone()){
        float[] array = new float[6];
        pi.currentSegment(array);
        float x = array[0];
        float y = array[1];
        pi.next();
    }



    return new DivingAction(info.getMaxAcceleration(), alignAcc);
}
}
