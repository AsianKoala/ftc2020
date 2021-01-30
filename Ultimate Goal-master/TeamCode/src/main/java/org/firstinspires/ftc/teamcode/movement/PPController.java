package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Point;
import org.firstinspires.ftc.teamcode.util.Pose;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.util.MathUtil.angleWrap;

public class PPController {

    public static final double smallAdjustSpeed = 0.135; // TODO: diff vals

    public enum profileStates {
        gunningIt,
        fineAdjustment;

        private static profileStates[] vals = values();
        public profileStates next(){
            return vals[(this.ordinal()+1) % vals.length];
        }
    }

    private static profileStates y_movement_state = profileStates.gunningIt;
    private static profileStates x_movement_state = profileStates.gunningIt;
    private static profileStates turn_movement_state = profileStates.gunningIt;


    public static void goToPosition(Pose currRobotPose, double targetX, double targetY, double point_angle, double movement_speed, double point_speed) {
        //get our distance away from the point
        double distanceToPoint = Math.sqrt(Math.pow(targetX-currRobotPose.x, 2) + Math.pow(targetY-currRobotPose.y,2));

        double angleToPoint = Math.atan2(targetY-currRobotPose.y,targetX-currRobotPose.x);
        double deltaAngleToPoint = angleWrap(angleToPoint-(currRobotPose.heading-Math.toRadians(90)));
        //x and y components required to move toward the next point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;

        double relative_abs_x = Math.abs(relative_x_to_point);
        double relative_abs_y = Math.abs(relative_y_to_point);



        //preserve the shape (ratios) of our intended movement direction but scale it by movement_speed
        double movement_x_power = (relative_x_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;
        double movement_y_power = (relative_y_to_point / (relative_abs_y+relative_abs_x)) * movement_speed;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(y_movement_state == profileStates.gunningIt) {
            if(relative_abs_y < 3){
                y_movement_state = y_movement_state.next();
            }
        }


        if(y_movement_state == profileStates.fineAdjustment){
            movement_y_power = Range.clip(((relative_y_to_point/8.0) * 0.15),-0.15,0.15);
        }

        if(x_movement_state == profileStates.gunningIt) {
            if(relative_abs_x < 3){
                x_movement_state = x_movement_state.next();
            }
        }

        if(x_movement_state == profileStates.fineAdjustment){
            movement_x_power = Range.clip(((relative_x_to_point/2.5) * smallAdjustSpeed),-smallAdjustSpeed,smallAdjustSpeed);
        }

        double rad_to_target = angleWrap(point_angle-currRobotPose.heading);
        double turnPower = 0;

        //every movement has two states, the fast "gunning" section and the slow refining part. turn this var off when close to target
        if(turn_movement_state == profileStates.gunningIt) {
            turnPower = rad_to_target > 0 ? point_speed : -point_speed;
            if(Math.abs(rad_to_target) < Math.toRadians(3.0)){
                turn_movement_state = turn_movement_state.next();
            }

        }


        if(turn_movement_state == profileStates.fineAdjustment){
            //this is a var that will go from 0 to 1 in the course of 10 degrees from the target
            turnPower = (rad_to_target/Math.toRadians(10)) * smallAdjustSpeed;
            turnPower = Range.clip(turnPower,-smallAdjustSpeed,smallAdjustSpeed);
        }

        DriveTrain.movementTurn = turnPower;
        DriveTrain.movementX = movement_x_power;
        DriveTrain.movementY = movement_y_power;
    }




    static class myPoint{
        public double x;
        public double y;
        public boolean onLine;
        public myPoint(double X, double Y, boolean isOnLine){
            x = X;
            y = Y;
            onLine = isOnLine;
        }
    }


    public static int followCurveIndex = 0;
    //NEED TO CALL TO START FOLLOW CURVE (call once)
    public static void initCurve(){
        followCurveIndex = 0;
    }


    /**
     * follows a set of points, while maintaining a following distance
     */
    public static boolean followCurve(ArrayList<CurvePoint> allPoints, double followAngle, boolean allowSkipping){

        //now we will extend the last line so that the pointing looks smooth at the end
        ArrayList<CurvePoint> pathExtended = (ArrayList<CurvePoint>) allPoints.clone();

        //first get which segment we are on
        pointWithIndex clippedToPath = clipToPath(allPoints,worldXPosition,worldYPosition);
        int currFollowIndex = clippedToPath.index+1;

        //get the point to follow
        CurvePoint followMe = getFollowPointPath(pathExtended,worldXPosition,worldYPosition,
                allPoints.get(currFollowIndex).followDistance);



        //this will change the last point to be extended
        pathExtended.set(pathExtended.size()-1,
                extendLine(allPoints.get(allPoints.size()-2),allPoints.get(allPoints.size()-1),
                        allPoints.get(allPoints.size()-1).pointLength * 1.5));



        //get the point to point to
        CurvePoint pointToMe = getFollowPointPath(pathExtended,worldXPosition,worldYPosition,
                allPoints.get(currFollowIndex).pointLength);



        //if we are nearing the end (less than the follow dist amount to go) just manualControl point to end
        //but only if we have passed through the correct points beforehand
        double clipedDistToFinalEnd = Math.hypot(
                clippedToPath.x-allPoints.get(allPoints.size()-1).x,
                clippedToPath.y-allPoints.get(allPoints.size()-1).y);



        if(clipedDistToFinalEnd <= followMe.followDistance + 15 ||
                Math.hypot(worldXPosition-allPoints.get(allPoints.size()-1).x,
                        worldYPosition-allPoints.get(allPoints.size()-1).y) < followMe.followDistance + 15){

            followMe.setPoint(allPoints.get(allPoints.size()-1).toPoint());
        }



        ComputerDebugging.sendKeyPoint(new Point(pointToMe.x,pointToMe.y));
        ComputerDebugging.sendKeyPoint(new Point(followMe.x,followMe.y));





        gunToPosition(followMe.x, followMe.y,followAngle,
                followMe.moveSpeed,followMe.turnSpeed,
                followMe.slowDownTurnRadians,0,true);

        //find the angle to that point using atan2
        double currFollowAngle = Math.atan2(pointToMe.y-worldYPosition,pointToMe.x-worldXPosition);

        //if our follow angle is different, point differently
        currFollowAngle += subtractAngles(followAngle,Math.toRadians(90));

        movementResult result = pointAngle(currFollowAngle,allPoints.get(currFollowIndex).turnSpeed,Math.toRadians(45));
        DriveTrain.movementX *= 1 - Range.clip(Math.abs(result.turnDelta_rad) / followMe.slowDownTurnRadians,0,followMe.slowDownTurnAmount);
        DriveTrain.movementY *= 1 - Range.clip(Math.abs(result.turnDelta_rad) / followMe.slowDownTurnRadians,0,followMe.slowDownTurnAmount);



        return clipedDistToFinalEnd < 10;//if we are less than 10 cm to the target, return true
    }


    /**
     * This will extend a line by a distance. It will modify only the second point
     * @param firstPoint
     * @param secondPoint
     * @return a new curve point that is a replacement for the second point
     */
    private static CurvePoint extendLine(CurvePoint firstPoint, CurvePoint secondPoint, double distance) {

        /**
         * Since we are pointing to this point, extend the line if it is the last line
         * but do nothing if it isn't the last line
         *
         * So if you imagine the robot is almost done its path, without this algorithm
         * it will just point to the last point on its path creating craziness around
         * the end (although this is covered by some sanity checks later).
         * With this, it will imagine the line extends further and point to a location
         * outside the endpoint of the line only if it's the last point. This makes the
         * last part a lot smoother, almost looking like a curve but not.
         */

        //get the angle of this line
        double lineAngle = Math.atan2(secondPoint.y - firstPoint.y,secondPoint.x - firstPoint.x);
        //get this line's length
        double lineLength = Math.hypot(secondPoint.x - firstPoint.x,secondPoint.y - firstPoint.y);
        //extend the line by 1.5 pointLengths so that we can still point to it when we
        //are at the end
        double extendedLineLength = lineLength + distance;

        CurvePoint extended = new CurvePoint(secondPoint);
        extended.x = Math.cos(lineAngle) * extendedLineLength + firstPoint.x;
        extended.y = Math.sin(lineAngle) * extendedLineLength + firstPoint.y;
        return extended;
    }


    /**
     * This is just a point with an index
     */
    public static class pointWithIndex{
        private double x;
        private double y;
        private int index;

        /**
         * initialize with the three things
         * @param xPos
         * @param yPos
         * @param index
         */
        public pointWithIndex(double xPos, double yPos, int index){
            this.x = xPos;
            this.y = yPos;
            this.index = index;
        }
    }

    /**
     * This will return which index along the path the robot is. It is the index of the first point
     * It also returns the point of the clipping
     * @param pathPoints
     * @param xPos
     * @param yPos
     * @return
     */
    public static pointWithIndex clipToPath(ArrayList<CurvePoint> pathPoints, double xPos, double yPos){
        double closestClippedDistance = 10000000;//start this off rediculously high

        //this is the index of the closest clipped distance
        //(index of the fist point on the line, not the second)
        int closestClippedIndex = 0;


        Point clippedToLine = new Point();//this is the clipped point where it is closest

        //go through all the points on the path
        for(int i = 0; i < pathPoints.size()-1; i ++){
            //get the two points on this segment
            CurvePoint firstPoint = pathPoints.get(i);
            CurvePoint secondPoint = pathPoints.get(i+1);

            //now we clip to the line
            Point currClippedToLine = clipToLine(firstPoint.x,firstPoint.y,
                    secondPoint.x,secondPoint.y, xPos,yPos);

            //get the distance to the clipped point
            double distanceToClip = Math.hypot(xPos-currClippedToLine.x,yPos-currClippedToLine.y);


            if(distanceToClip < closestClippedDistance){
                closestClippedDistance = distanceToClip;//save this as a new record low
                closestClippedIndex = i;//save the index of the closest clip distance
                clippedToLine = currClippedToLine;//save this
            }
        }
        //return the three things
        return new pointWithIndex(clippedToLine.x,clippedToLine.y,closestClippedIndex);//now return the closestClippedIndex
    }




    /**
     * This will return a follow point when given a path
     * @param pathPoints all the points on the path
     * @param xPos x position of the robot
     * @param yPos y position of the robot
     * @return the point the robot should move follow or move towards
     */
    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints,
                                                double xPos,
                                                double yPos,
                                                double followRadius){

        /**
         * Step 1: figure out what segment we are on by closest clipped to line distance
         */
        pointWithIndex clippedToLine = clipToPath(pathPoints,xPos,yPos);
        int currFollowIndex = clippedToLine.index;//this is the index of the first point on the path we are following



        /**
         * Step 2: extend circle from robot pos - intersect with each segment ->
         * follow closest to last point (ADD BETTER CHOOSING ALGORITHM)
         */
        //copy this so we don't change the path
        CurvePoint followMe = new CurvePoint(pathPoints.get(currFollowIndex+1));
        //by default go to the follow point
        followMe.setPoint(new Point(clippedToLine.x,clippedToLine.y));

        //go through all the path points and intersect the circle
        for(int i = 0; i < pathPoints.size()-1; i ++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            //get the intersections with this line
            ArrayList<Point> intersections =
                    MathUtil.lineCircleIntersection(xPos,yPos,followRadius,
                            startLine.x,startLine.y,endLine.x,endLine.y);

            //go through all the intersection points and see which is closest to the destination
            double closestDistance = 1000000;
            for(int p = 0; p < intersections.size(); p ++){

                Point thisIntersection = intersections.get(p);

                double dist = Math.hypot(thisIntersection.x - pathPoints.get(pathPoints.size()-1).x,
                        thisIntersection.y - pathPoints.get(pathPoints.size()-1).y);

                //follow if the distance to the last point is less than the closestDistance
                if(dist < closestDistance){
                    closestDistance = dist;
                    followMe.setPoint(thisIntersection);//set the point to the intersection
                }
            }
        }


        return followMe;
    }




    //takes any point and finds where it should be on the line (closest point on the line)
    public static Point clipToLine(double lineX1, double lineY1, double lineX2, double lineY2,
                                   double robotX, double robotY){
        if(lineX1 == lineX2){
            lineX1 = lineX2 + 0.01;//nah
        }
        if(lineY1 == lineY2){
            lineY1 = lineY2 + 0.01;//nah
        }

        //calculate the slope of the line
        double m1 = (lineY2 - lineY1)/(lineX2 - lineX1);
        //calculate the slope perpendicular to this line
        double m2 = (lineX1 - lineX2)/(lineY2 - lineY1);

        //clip the robot's position to be on the line
        double xClipedToLine = ((-m2*robotX) + robotY + (m1 * lineX1) - lineY1)/(m1-m2);
        double yClipedToLine = (m1 * (xClipedToLine - lineX1)) + lineY1;
        return new Point(xClipedToLine,yClipedToLine);
    }

    //extends a point on a line in the direction of the two points defining the line (order matters)
    //this also clips it
    public static Point extendPointOnLine(double lineX1, double lineY1, double lineX2, double lineY2,
                                          double pointX, double pointY, double extendDist){
        Point cliped = clipToLine(lineX1,lineY1,lineX2,lineY2,pointX,pointY);

        //make sure we are not exactly on a 90 degree interval
        if(lineX1 == lineX2){
            lineX1 += 0.0001;
        }
        if(lineY1 == lineY2){
            lineY1 += 0.0001;
        }
        //this is the angle the line forms with the x axis
        double angleLine = Math.atan2(lineY2 - lineY1,lineX2 - lineX1);


        //extend our line
        double xTarget = (Math.cos(angleLine) * extendDist) + cliped.x;
        double yTarget = (Math.sin(angleLine) * extendDist) + cliped.y;


        return new Point(xTarget,yTarget);
    }
    public static myPoint pointAlongLine(double lineX1, double lineY1, double lineX2, double lineY2,
                                         double robotX, double robotY,
                                         double followDistance){
        Point clipedToLine = clipToLine(lineX1, lineY1, lineX2, lineY2, robotX, robotY);



        //this is the angle the line forms with the x axis
        double angleLine = Math.atan2(lineY2 - lineY1,lineX2 - lineX1);


        //Calculate the follow point
        //we know that the point order is correct and atan2 means we don't have
        //to worry about quadrants
        double xTarget = (Math.cos(angleLine) * followDistance) + clipedToLine.x;
        double yTarget = (Math.sin(angleLine) * followDistance) + clipedToLine.y;


        //FIGURE OUT IT WE ARE ACTUALLY ON THE LINE SEGMENT
        boolean pointIsOnLine = false;
        //make sure the point is not out of range of the line
        if((xTarget > lineX2 && lineX2 < lineX1) || (xTarget < lineX2 && lineX2 > lineX1)){
            pointIsOnLine = true;
        }

        return new myPoint(xTarget,yTarget,pointIsOnLine);
    }

}