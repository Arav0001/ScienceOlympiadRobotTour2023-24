struct Pose2D {
    float x, y, theta;

    Pose2D(float x, float y, float theta) : x(x), y(y), theta(theta) {};
    Pose2D() : x(0), y(0), theta(0) {};

    Pose2D operator+(const Pose2D& pose) {
        Pose2D result;
        
        result.x = x + pose.x;
        result.y = y + pose.y;
        result.theta = theta + pose.theta;

        return result;
    }

    Pose2D operator-(const Pose2D& pose) {
        Pose2D result;
        
        result.x = x - pose.x;
        result.y = y - pose.y;
        result.theta = theta - pose.theta;

        return result;
    }
};