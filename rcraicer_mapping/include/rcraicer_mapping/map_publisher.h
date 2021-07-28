#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rcraicer_msgs/msg/sim_state.hpp"

#include <image_transport/image_transport.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/publisher.hpp>

 #include <cv_bridge/cv_bridge.h>

#include <vector>
#include <unordered_map>
#include <algorithm>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "opencv2/opencv.hpp"
#include "cnpy.h"

# define M_PI		3.14159265358979323846	/* pi */
# define M_PI_2		1.57079632679489661923	/* pi/2 */

#define COS_45      0.7071

typedef struct {
	float x;
    float y;
    float z;
    float r;
    float g;
    float b;
} costmap_point_t ;

typedef struct {
    int minX;
    int maxX;
    int minY;
    int maxY;
} costmap_dimensions_t;

// typedef struct {
//     float x;
//     float y;
// } costmap_key_t;

template<typename T1, typename T2>
struct costmap_key {
    T1 x;
    T2 y;

    costmap_key(T1 x, T2 y)
    {
        this->x = x;
        this->y = y;
    }

    bool operator==(const costmap_key &p) const {
        return x == p.x && y == p.y;
    }    
};

struct costmap_hash_fn
{
    template <class T1, class T2>
    std::size_t operator() (const costmap_key<T1, T2> &key) const
    {
        std::size_t h1 = std::hash<T1>()(key.x);
        std::size_t h2 = std::hash<T2>()(key.y);

        return h1 ^ h2;
    }
};

struct costmap_equal_fn
{
    template <class T1, class T2>
    bool operator() (const costmap_key<T1, T2> &lhs, const costmap_key<T1, T2> &rhs) const
    {        
        return lhs.x == rhs.x && lhs.y && rhs.y;
    }
};

typedef union {
	uint8_t data[24];
	struct {
		float x;
        float y;
        float z;
        float r;
        float g;
        float b;
	};
} costmap_vec_point_t;

class MapPublisher : public rclcpp::Node
{
    public:
        MapPublisher();
        ~MapPublisher();        

    private:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPublisher;        
        image_transport::Publisher imagePublisher;        

        rclcpp::Subscription<rcraicer_msgs::msg::SimState>::SharedPtr ssSubscription;

        rclcpp::TimerBase::SharedPtr publishTimer;
        
        void sim_state_callback(const rcraicer_msgs::msg::SimState::SharedPtr msg);
        void updateInternalParams();        

        void convertPoint(float x, float y, int &xOut, int &yOut);
        void roundPoints(float x, float y, float &xOut, float &yOut);
        void transformPoint(float x, float y, float distance, float heading, float &xOut, float &yOut);
        void addPointFields();
        int addPointField(std::string name, int datatype, int count, int offset);
        void addToPointCloud(float x, float y, float heading);
        bool addPointToPointCloud(float x, float y, float offset);
        void getRGBValue(float offset, costmap_vec_point_t &point);
        void updateMinMax(float x, float y);
        void loadFromFile();
        void publishImage(int x, int y, float yaw);

        void publishPointCloud();        
        float calculateDistance(float x1, float y1, float x2, float y2);

        rcl_interfaces::msg::SetParametersResult paramSetCallback(const std::vector<rclcpp::Parameter>& parameters);
        OnSetParametersCallbackHandle::SharedPtr paramSetCallbackHandler;        

        sensor_msgs::msg::PointCloud2 pointCloud;

        int createButtonId {5}; // right bumper
        int deleteButtonId {4}; // left bumper
        int saveButtonId {2}; // x   y #3

        bool createMode {false};
        double createButtonStamp = {0.0};
        bool gotFirstPoint {false};        
        double lastX {0};
        double lastY {0};
        double lastHeading {0.0};
        int pixelsPerMeter {10};
        double mapDensity {0.1};
        double halfMapDensity {0.05};
        double trackWidth {1.8};            
        int trackStepCount {0};               
        double offsetStep {0.0};
        int pixelBorder {20};

        int xOffset {0};
        int yOffset {0};

        int xLength {0};
        int yLength {0};

        cv::Size intermediateImageSize;
        cv::Size outputImageSize;

        cv::Rect intermediateImageDestRect;
        cv::Rect outputImageSrcRect;
        cv::Rect outputImageDestRect;
        
        costmap_dimensions_t cm_dims {0, 0, 0, 0};
        bool cm_dims_init {true};

        std::string imageFileName = "costmap.png";
        std::string npzFileName = "costmap.npz";

        cv::Mat costmapImage;
        cv::Mat outputImage;
                
        std::unordered_map<costmap_key<int, int>, costmap_vec_point_t, costmap_hash_fn> mapPoints;        

};