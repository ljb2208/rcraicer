#include "../include/rcraicer_mapping/map_generator.h"

using std::placeholders::_1;
using namespace std::chrono_literals;


MapGenerator::MapGenerator() : Node("map_generator_node")
{
    paramSetCallbackHandler = this->add_on_set_parameters_callback(std::bind(&MapGenerator::paramSetCallback, this, std::placeholders::_1));

    updateInternalParams();
 
    pcPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cost_map", 10);        

    joySubscription = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MapGenerator::joy_callback, this, std::placeholders::_1));   

    ssSubscription = this->create_subscription<rcraicer_msgs::msg::SimState>(
      "sim_state", 10, std::bind(&MapGenerator::sim_state_callback, this, std::placeholders::_1));       

    publishTimer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapGenerator::publishPointCloud, this));        


    pointCloud.height = 1;
    pointCloud.width = 0;
    pointCloud.header.frame_id = "map";

    pointCloud.point_step = sizeof(costmap_vec_point_t);
    pointCloud.is_bigendian = false;
    pointCloud.is_dense = false;   

    addPointFields();     

    RCLCPP_INFO(this->get_logger(), "Node started.");    
}

MapGenerator::~MapGenerator()
{
}


rcl_interfaces::msg::SetParametersResult MapGenerator::paramSetCallback(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (auto param : parameters)
    {        
    }

    updateInternalParams();
    return result;
}


void MapGenerator::updateInternalParams()
{
    mapDensity = 1.0 / pixelsPerMeter;
    halfMapDensity = mapDensity / 4;    
    trackStepCount = trackWidth / halfMapDensity;
    offsetStep = 0.8 / trackStepCount;    
    pixelOffset = pixelBorder / 2 * mapDensity;
}

void MapGenerator::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)  // use const at end of function in later versions of ros2
{    
    double ts = this->get_clock()->now().seconds();

    if (!createMode && msg->buttons[createButtonId] == 1 && (ts - createButtonStamp) > 1.0)
    {
        RCLCPP_INFO(this->get_logger(), "Mapping turned on");
        createMode = true;
        createButtonStamp = ts;
    }
    else if (createMode && msg->buttons[createButtonId] == 1 && (ts - createButtonStamp) > 1.0)
    {
        RCLCPP_INFO(this->get_logger(), "Mapping turned off");
        createMode = false;
        createButtonStamp = ts;
    }

    if (msg->buttons[deleteButtonId] == 1)
    {

    }

    if (msg->buttons[saveButtonId] == 1)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map");
        saveToFile();
    }    
}

void MapGenerator::sim_state_callback(const rcraicer_msgs::msg::SimState::SharedPtr msg)
{
    if (!createMode)
        return;
        
    double roll, pitch, yaw;
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    float distance = calculateDistance(lastX, lastY, msg->env_position.x, msg->env_position.y);

    // interpolate if distance travelled greater than map density
    if (gotFirstPoint && distance > halfMapDensity)
    {
        int steps = distance / halfMapDensity;
        float deltaHeading = (yaw - lastHeading ) / steps;
        float xIn = lastX;
        float yIn = lastY;

        for (int i=0; i < steps; i++)
        {            
            float xOut, yOut;
            float interpHeading = lastHeading + deltaHeading * steps;
            transformPoint(xIn, yIn, halfMapDensity, interpHeading, xOut, yOut);

            if (calculateDistance(xOut, yOut, msg->env_position.x, msg->env_position.y) <= halfMapDensity)
            {
                break;
            }

            addToPointCloud(xOut, yOut, interpHeading);
            xIn = xOut;
            yIn = yOut;
        }
    }

    if (!gotFirstPoint)
    {
        startX = msg->env_position.x;
        startY = msg->env_position.y;
        gotFirstPoint = true;
    }

    addToPointCloud(msg->env_position.x, msg->env_position.y, yaw);

    lastX = msg->env_position.x;
    lastY = msg->env_position.y;
    lastHeading = yaw;
}

void MapGenerator::roundPoints(float x, float y, float &xOut, float &yOut)
{
    // xOut = roundf(x / halfMapDensity) * halfMapDensity;
    // yOut = roundf(y / halfMapDensity) * halfMapDensity;

    xOut = roundf(x * pixelsPerMeter) / pixelsPerMeter;
    yOut = roundf(y * pixelsPerMeter) / pixelsPerMeter;
}

void MapGenerator::convertPoint(float x, float y, int &xOut, int &yOut)
{
    xOut = (int) roundf(x * pixelsPerMeter);
    yOut = (int) roundf(y * pixelsPerMeter);
}

void MapGenerator::transformPoint(float x, float y, float distance, float heading, float &xOut, float &yOut)
{
    // float xnew = x + (distance * cos(heading));
    // float ynew = y + (distance * sin(heading));

    xOut = x + (distance * cos(heading));
    yOut = y + (distance * sin(heading));

    // roundPoints(xnew, ynew, xOut, yOut);
}

float MapGenerator::calculateDistance(float x1, float y1, float x2, float y2)
{
    float deltax = x1 - x2;
    float deltay = y1 - y2;
    return sqrt(deltax* deltax + deltay * deltay);
}

void MapGenerator::addToPointCloud(float x, float y, float heading)
{
    // float rx, ry;
    // roundPoints(x, y, rx, ry);    

    float rx, ry;
    rx = x;
    ry = y;

    // add initial point to cloud
    float offset = 0.0;
    addPointToPointCloud(rx, ry, offset);

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

bool MapGenerator::addPointToPointCloud(float x, float y, float offset)
{ 
    int rx, ry;
    convertPoint(x, y, rx, ry);
    costmap_key<int, int> key(rx, ry);
    
    // if point exists then return
    if (mapPoints.find(key) != mapPoints.end())
         return false;

    // if (offset == 0.0)
    //     std::cout << "Add point x: " << x << " Y: " << y << " offset: " << offset << "\n";    

    updateMinMax(rx, ry);

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

void MapGenerator::updateMinMax(float x, float y)
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

void MapGenerator::getRGBValue(float offset, costmap_vec_point_t &point)
{
    float colorVal = 1.0 - offset;
    point.r = colorVal;
    point.g = colorVal;
    point.b = colorVal;
}

void MapGenerator::publishPointCloud()
{
    pointCloud.width = mapPoints.size();    
    pointCloud.header.stamp = this->get_clock()->now();

    pcPublisher->publish(pointCloud);
}

void MapGenerator::addPointFields()
{
    int offset = addPointField("x", 7, 1, 0);
    offset = addPointField("y", 7, 1, offset);
    offset = addPointField("z", 7, 1, offset);
    offset = addPointField("rgb", 7, 3, offset);
}

int MapGenerator::addPointField(std::string name, int datatype, int count, int offset)
{
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.datatype = datatype;
    field.count = count;
    field.offset = offset;

    pointCloud.fields.push_back(field);    

    return offset + sizeof(float);
}

void MapGenerator::saveToFile()
{
    // float xBounds = (cm_dims.maxX - cm_dims.minX) * pixelsPerMeter + pixelBorder;
    // float yBounds = (cm_dims.maxY - cm_dims.minY) * pixelsPerMeter + pixelBorder;

    // bounds in pixels
    int xBounds = cm_dims.maxX - cm_dims.minX;
    int yBounds = cm_dims.maxY - cm_dims.minY;

    // add pixel border in meters
    xBounds += pixelBorder;
    yBounds += pixelBorder;
     
    // calculate number of steps in pixels
    int xSteps = xBounds;
    int ySteps = yBounds;

    int xBoundsNPZ[2];
    int yBoundsNPZ[2];

    // calculate bounds in pixels for numpy array
    xBoundsNPZ[0] = cm_dims.minX - (pixelBorder / 2);
    xBoundsNPZ[1] = xBoundsNPZ[0] + xBounds;

    yBoundsNPZ[0] = cm_dims.minY - (pixelBorder / 2);
    yBoundsNPZ[1] = yBoundsNPZ[0] + yBounds;


    cnpy::npz_save("costmap.npz", "pixelsPerMeter", &pixelsPerMeter, {1}, "w");
    cnpy::npz_save("costmap.npz", "xBounds", &xBoundsNPZ[0], {2}, "a");
    cnpy::npz_save("costmap.npz", "yBounds", &yBoundsNPZ[0], {2}, "a");

    cv::Mat img(ySteps, xSteps, CV_8UC3, cv::Scalar(0, 0, 0));

    int numPoints = 0;

    std::vector<float> channel0;    

    int rminX = xBoundsNPZ[0];
    int rminY = yBoundsNPZ[0];

    int xcurr = rminX;
    int ycurr = rminY;

    // int xtest, ytest;

    // convertPoint(0.0, 0.0, xtest, ytest);

    // auto testpoint = mapPoints.find(costmap_key<int, int>(xtest, ytest));

    // if (testpoint != mapPoints.end())
    // {
    //     std::cout << "testpoint: " << testpoint->second.r << "\n";
    // }

    for (int y=0; y < ySteps; y++)
    {
        xcurr = rminX;        

        for (int x=0; x < xSteps; x++)
        {
            costmap_key<int, int> key(xcurr, ycurr);      

            // if (xcurr == xtest && ycurr == ytest)      
            // {
            //     std::cout << "test point output y: " << y << " x: " << x << "\n";
            // }

            auto point = mapPoints.find(key);

            if (point != mapPoints.end())
            {
                float r = point->second.r;                

                img.at<cv::Vec3b>(y, x)[0] = r * 255;                               
                img.at<cv::Vec3b>(y, x)[1] = r * 255;                               
                img.at<cv::Vec3b>(y, x)[2] = r * 255;                               
                
                channel0.push_back(1.0 - r);                

                numPoints++;                
            }
            else
            {
                channel0.push_back(1.1);                
            }

            xcurr++;
        }

        ycurr++;
    }

    cnpy::npz_save("costmap.npz", "channel0", channel0.data(), {channel0.size()}, "a");    

    // flip image around x axis
    cv::Mat flipImg(ySteps, xSteps, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::flip(img, flipImg, 0);
    cv::imwrite("costmap.png", flipImg);

    RCLCPP_INFO(this->get_logger(), "Map saved. %i valid points. %i total points", numPoints, channel0.size());

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGenerator>());
    rclcpp::shutdown();
    return 0;
}