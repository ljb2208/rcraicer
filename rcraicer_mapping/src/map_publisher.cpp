#include "../include/rcraicer_mapping/map_publisher.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


MapPublisher::MapPublisher() : Node("map_publisher_node")
{
    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&MapPublisher::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();
 
    pcPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cost_map", 10);        
    // imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("cm_image", 10);
    imagePublisher = image_transport::create_publisher(this, "cm_image");

    ssSubscription = this->create_subscription<rcraicer_msgs::msg::SimState>(
      "sim_state", 10, std::bind(&MapPublisher::sim_state_callback, this, std::placeholders::_1));       

    publishTimer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapPublisher::publishPointCloud, this));        

    pointCloud.height = 1;
    pointCloud.width = 0;
    pointCloud.header.frame_id = "map";

    pointCloud.point_step = sizeof(costmap_vec_point_t);
    pointCloud.is_bigendian = false;
    pointCloud.is_dense = false;   

    outputImageSize.width = 160;
    outputImageSize.height = 160;

    intermediateImageSize.width = (outputImageSize.width / COS_45) * 2;
    intermediateImageSize.height = (outputImageSize.height / COS_45) * 2;

    outputImage = cv::Mat(outputImageSize.height, outputImageSize.width, CV_8UC3, cv::Scalar(0, 0, 0));

    intermediateImageDestRect = cv::Rect(cv::Point(0, 0), cv::Point(intermediateImageSize.height, intermediateImageSize.width));

    outputImageSrcRect = cv::Rect(cv::Point(intermediateImageSize.width/2 - outputImageSize.width/2, intermediateImageSize.height/2 - outputImageSize.height), cv::Point(intermediateImageSize.width/2 + outputImageSize.width/2, intermediateImageSize.height/2));    
    outputImageDestRect = cv::Rect(cv::Point(0, 0), cv::Point(outputImageSize.height, outputImageSize.width));

    addPointFields();     

    loadFromFile();

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

MapPublisher::~MapPublisher()
{
}


rcl_interfaces::msg::SetParametersResult MapPublisher::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto param : parameters)
    {        
    }

    updateInternalParams();
    return result;
}


void MapPublisher::updateInternalParams()
{
    mapDensity = 1.0 / pixelsPerMeter;
    halfMapDensity = mapDensity / 4;    
    trackStepCount = trackWidth / halfMapDensity;
    offsetStep = 0.8 / trackStepCount;    
}

void MapPublisher::sim_state_callback(const rcraicer_msgs::msg::SimState::SharedPtr msg)
{    

    if (!gotFirstPoint)
    {
        gotFirstPoint = true;
    }
    else
    {
        if (calculateDistance(msg->env_position.x, msg->env_position.y, lastX, lastY) < 0.1)
        {
            return;
        }        
    }

    lastX = msg->env_position.x;
    lastY = msg->env_position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    int x, y;
    convertPoint(msg->env_position.x, msg->env_position.y, x, y);

    x = x - xOffset;
    y = yLength - (y - yOffset);    

    publishImage(x, y, yaw);
}

void MapPublisher::publishImage(int x, int y, float yaw)
{
    if (x < 0 || y < 0 || x >= xLength || y >= yLength)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid data");
        return;
    }

    // cv::circle(costmapImage, cv::Point(x, y), 20, cv::Scalar(255, 0, 0),2);    
    // cv::imwrite("circle.png", costmapImage);

    int leftBound = x - intermediateImageSize.width/2; 
    int rightBound = x + intermediateImageSize.width/2;

    int topBound = y - intermediateImageSize.height/2;
    int bottomBound = y + intermediateImageSize.height/2;     

    int leftBound2 = 0;
    int rightBound2 = intermediateImageSize.width;

    int topBound2 = 0;
    int bottomBound2 = intermediateImageSize.height;

    // check bounds
    if (leftBound < 0)
    {
        leftBound2 = abs(leftBound);
        leftBound = 0;        
    }
    
    if (rightBound >= xLength)
    {
        rightBound2 = rightBound2 - (rightBound - xLength) - 1;
        rightBound = xLength - 1;        
    }

    if (topBound < 0)
    {
        topBound2 = abs(topBound);
        topBound = 0;
    }
    
    if (bottomBound >= yLength)
    {
        bottomBound2 = bottomBound2 - (bottomBound - yLength) - 1;
        bottomBound = yLength - 1;
    }


    cv::Rect srcRect(cv::Point(leftBound, topBound), cv::Point(rightBound, bottomBound));
    cv::Rect destRect(cv::Point(leftBound2, topBound2), cv::Point(rightBound2, bottomBound2));
    // cv::Rect destRect(cv::Point(abs(x-intermediateImageSize.width/2 - leftBound), abs(y - intermediateImageSize.height/2 - topBound)), cv::Point(rightBound - leftBound, bottomBound - topBound));
    // cv::Rect srcRect(cv::Point(x - intermediateImageSize.width/2, y - intermediateImageSize.height/2), cv::Point(x + intermediateImageSize.width/2, y + intermediateImageSize.height/2));        


    // cv::Rect destRect(cv::Point(0, 0), cv::Point(intermediateImageSize.height, intermediateImageSize.width));
    cv::Mat image(intermediateImageSize.height, intermediateImageSize.width, CV_8UC3, cv::Scalar(0, 0, 0));    
    costmapImage(srcRect).copyTo(image(destRect));

    // cv::imwrite("test-prerot.png", image);

    cv::Mat rotImage(intermediateImageSize.height, intermediateImageSize.width, CV_8UC3, cv::Scalar(0, 0, 0));    

    cv::Mat rmat = cv::getRotationMatrix2D(cv::Point2f(intermediateImageSize.height/2, intermediateImageSize.width/2), (-yaw + M_PI_2) * 180.0 / M_PI, 1.0);    
    cv::warpAffine(image, rotImage, rmat, intermediateImageSize, cv::INTER_CUBIC);

    // cv::imwrite("test-rot.png", rotImage);

    // srcRect = cv::Rect(cv::Point(intermediateImageSize.width/2 -outputImageSize.width/2, intermediateImageSize.height/2), cv::Point(intermediateImageSize.width/2 -outputImageSize.width/2 + outputImageSize.width, intermediateImageSize.height/2 - outputImageSize.height/2));    
    // srcRect = cv::Rect(cv::Point(intermediateImageSize.width/2 - outputImageSize.width/2, intermediateImageSize.height/2 - outputImageSize.height), cv::Point(intermediateImageSize.width/2 + outputImageSize.width/2, intermediateImageSize.height/2));    
    rotImage(outputImageSrcRect).copyTo(outputImage);

    // cv::imwrite("output.png", outputImage);

    cv_bridge::CvImage cvrImage;
    cvrImage.header.stamp = this->get_clock()->now();
    cvrImage.header.frame_id = "map";
    cvrImage.encoding = "bgr8";
    cvrImage.image = cv::Mat(outputImage.cols, outputImage.rows, CV_8UC3);
    outputImage.copyTo(cvrImage.image);
    sensor_msgs::msg::Image rosImage;
    cvrImage.toImageMsg(rosImage);

    imagePublisher.publish(rosImage);
    // cv::imwrite("test.png", rotImage);
    // cv::imwrite("output.png", outputImage);


}

void MapPublisher::roundPoints(float x, float y, float &xOut, float &yOut)
{
    // xOut = roundf(x / halfMapDensity) * halfMapDensity;
    // yOut = roundf(y / halfMapDensity) * halfMapDensity;

    xOut = roundf(x * pixelsPerMeter) / pixelsPerMeter;
    yOut = roundf(y * pixelsPerMeter) / pixelsPerMeter;
}

void MapPublisher::convertPoint(float x, float y, int &xOut, int &yOut)
{
    xOut = (int) roundf(x * pixelsPerMeter);
    yOut = (int) roundf(y * pixelsPerMeter);
}

void MapPublisher::transformPoint(float x, float y, float distance, float heading, float &xOut, float &yOut)
{
    // float xnew = x + (distance * cos(heading));
    // float ynew = y + (distance * sin(heading));

    xOut = x + (distance * cos(heading));
    yOut = y + (distance * sin(heading));

    // roundPoints(xnew, ynew, xOut, yOut);
}

float MapPublisher::calculateDistance(float x1, float y1, float x2, float y2)
{
    float deltax = x1 - x2;
    float deltay = y1 - y2;
    return sqrt(deltax* deltax + deltay * deltay);
}

void MapPublisher::addToPointCloud(float x, float y, float heading)
{
    // float rx, ry;
    // roundPoints(x, y, rx, ry);    

    float rx, ry;
    rx = x;
    ry = y;

    // add initial point to cloud
    float offset = 0.0;
    if (addPointToPointCloud(rx, ry, offset))
    {
        int dx, dy;
        convertPoint(x, y, dx, dy);
        std::cout << "Point added x: " << rx << "/" << dx << " y: " << ry << "/" << dy << "\r\n";
    }

    float xp, yp, hp;

    hp = heading + M_PI_2;

    // add points to left of initial point
    for (int i=1; i <= trackStepCount; i++)
    {
        transformPoint(rx, ry, i * halfMapDensity, hp, xp, yp);
        addPointToPointCloud(xp, yp, offset + offsetStep * i);
    }

    hp = heading - M_PI_2;

    // add points to right of initial point
    for (int i=1; i <= trackStepCount; i++)
    {
        transformPoint(rx, ry, i * halfMapDensity, hp, xp, yp);
        addPointToPointCloud(xp, yp, offset + offsetStep * i);
    }
}

bool MapPublisher::addPointToPointCloud(float x, float y, float offset)
{
    int rx, ry;
    convertPoint(x, y, rx, ry);
    costmap_key<int, int> key(rx, ry);
    
    // if point exists then return
    if (mapPoints.find(key) != mapPoints.end())
         return false;

    updateMinMax(x, y);

    costmap_vec_point_t point;
    // point.x = x;
    // point.y = y;
    point.x = rx / 10.0;
    point.y = ry / 10.0;
    point.z = 0.0;

    getRGBValue(offset, point);

    for (size_t i=0; i < sizeof(costmap_vec_point_t); i++)
    {
        pointCloud.data.push_back(point.data[(int)i]);
    }
    
    mapPoints.insert({key, point});    
    return true;
}

void MapPublisher::updateMinMax(float x, float y)
{
    if (cm_dims_init)
    {
        cm_dims.minX = x;
        cm_dims.maxX = x;
        cm_dims.minY = y;
        cm_dims.maxY = y;
        cm_dims_init = false;
    }
    else
    {
        if (x > cm_dims.maxX)
            cm_dims.maxX = x;
        if (x < cm_dims.minX)
            cm_dims.minX = x;

        if (y > cm_dims.maxY)
            cm_dims.maxY = y;
        if (y < cm_dims.minY)
            cm_dims.minY = y;
    }
}

void MapPublisher::getRGBValue(float offset, costmap_vec_point_t &point)
{
    float colorVal = 1.0 - offset;
    point.r = colorVal;
    point.g = colorVal;
    point.b = colorVal;
}

void MapPublisher::publishPointCloud()
{
    pointCloud.width = mapPoints.size();    
    pointCloud.header.stamp = this->get_clock()->now();

    pcPublisher->publish(pointCloud);
}

void MapPublisher::addPointFields()
{
    int offset = addPointField("x", 7, 1, 0);
    offset = addPointField("y", 7, 1, offset);
    offset = addPointField("z", 7, 1, offset);
    offset = addPointField("rgb", 7, 3, offset);
}

int MapPublisher::addPointField(std::string name, int datatype, int count, int offset)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.datatype = datatype;
    field.count = count;
    field.offset = offset;

    pointCloud.fields.push_back(field);    

    return offset + sizeof(float);
}

void MapPublisher::loadFromFile()
{
    // load image file
    costmapImage = cv::imread(imageFileName);

    xLength = costmapImage.cols;
    yLength = costmapImage.rows;

    // load npz file
    cnpy::npz_t map_dict = cnpy::npz_load(npzFileName);

    float* xBounds = map_dict["xBounds"].data<float>();
    float* yBounds = map_dict["yBounds"].data<float>();
    int* ppm = map_dict["pixelsPerMeter"].data<int>();
    
    cm_dims.minX = xBounds[0] - 1;
    cm_dims.maxX = xBounds[1] + 1;
    cm_dims.minY = yBounds[0] - 1;
    cm_dims.maxY = yBounds[1] + 1;
    pixelsPerMeter = ppm[0];  

    xOffset = round(cm_dims.minX * pixelsPerMeter);
    yOffset = round(cm_dims.minY * pixelsPerMeter);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPublisher>());
    rclcpp::shutdown();
    return 0;
}
