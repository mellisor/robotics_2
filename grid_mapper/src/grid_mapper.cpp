#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

// #define STAGE
// #define ODOM
using namespace boost::posix_time;

class GridMapper {
public:
  // Construst a new occupancy grid mapper  object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  GridMapper(ros::NodeHandle& nh, int width, int height, char set, int scale) :

    canvas(height, width, CV_8UC1) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    // If stage is defined, use these topics
    if( set == 's') {
      laserSub = nh.subscribe("base_scan", 1, \
        &GridMapper::laserCallback, this);
      // Subscribe to the current simulated robot' ground truth pose topic
      // and tell ROS to call this->poseCallback(...) whenever a new
      // message is published on that topic
      poseSub = nh.subscribe("base_pose_ground_truth", 1, \
        &GridMapper::poseCallbackNav, this);
    }
    // Otherwise use the rosbag topics
    else {
      laserSub = nh.subscribe("scan", 1, \
        &GridMapper::laserCallback, this);
    }
    if(set == 'o') {
      std::cout << "HI" << std::endl;
      poseSub = nh.subscribe("/odom", 1, \
        &GridMapper::poseCallbackNav, this);
    }
    else if(set == 'c') {
      poseSub = nh.subscribe("/odom_combined", 1, \
        &GridMapper::poseCallbackGeo, this);
    }

    // Create resizeable named window
    cv::namedWindow("Occupancy Grid Canvas", \
      CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);

    GRID_SCALE = scale;
    grid = std::vector<std::vector<float>>(width);
    for(int i = 0; i < width; i++)
    {
      grid[i] = std::vector<float>(height);
      for(int j = 0; j < height; j++) {
        grid[i][j] = 50.0f;
      }
    }
  };  
  
  // Save a snapshot of the occupancy grid canvas
  // NOTE: image is saved to same folder where code was executed
  void saveSnapshot() {
    std::string filename = "grid_" + to_iso_string(second_clock::local_time()) + ".jpg";
    canvasMutex.lock();
    cv::imwrite(filename, canvas);
    canvasMutex.unlock();
  };
  
  
  // Update grayscale intensity on canvas pixel (x, y) (in robot coordinate frame)
  void plot(int x, int y, char value) {
    canvasMutex.lock();
    x+=canvas.rows/2;
    y+=canvas.cols/2;
    if (x >= 0 && x < canvas.rows && y >= 0 && y < canvas.cols) {
      canvas.at<char>(x, y) = value;
    }
    canvasMutex.unlock();
  };

  // Update grayscale intensity on canvas pixel (x, y) (in image coordinate frame)
  void plotImg(int x, int y, char value) {
    canvasMutex.lock();
    if (x >= 0 && x < canvas.cols && y >= 0 && y < canvas.rows) {
      canvas.at<char>(y, x) = value;
    }
    canvasMutex.unlock();
  };

  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };

  // Checks whether given coordinates are in canvas boundaries
  bool inBounds(int x, int y) {
    return ( x > 0 && y > 0 && x < canvas.cols && y < canvas.rows);
  }

  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data and update occupancy grid canvas
    //       (use CELL_OCCUPIED, CELL_UNKNOWN, CELL_FREE, and CELL_ROBOT values)
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)

    // MY STUFF
    float maxIndex = floor((msg->angle_max - msg->angle_min) / msg->angle_increment);
    float scanAngle = 0.0f;
    float range = 0.0f;
    // For each angle increment
    for (unsigned int currIndex = 0; currIndex < maxIndex; currIndex++) {
      // Get angle of the scan 
      scanAngle = heading + msg->angle_min + currIndex*msg->angle_increment;
      // Get range of nearest obstacle at angle
      range = msg->ranges[currIndex]*GRID_SCALE;
      // Choose number of increments for line drawing
      int increments = ceil(range);
      // Get obstacle position
      float obstacle_x = x + cos(scanAngle)*range;
      float obstacle_y = y + sin(scanAngle)*range;
      // For line drawing, get dy/dx
      float dy = (obstacle_y - y) / increments;
      float dx = (obstacle_x - x) / increments;
      // Draw a free cell at each increment on the line
      for(int i = 0; i < increments; i++) {
        float currX = x + i*dx;
        float currY = y + i*dy;
        // Get canvas coordinates for vectors
        int canX = floor(currX + canvas.cols/2);
        int canY = floor(currY + canvas.rows/2);
        if(inBounds(canX,canY)) {
          // Update probability of cell occupancy
          grid[canX][canY] = grid[canX][canY]*smp_sig / samples;
          if(grid[canX][canY] < 50.0f) 
            plot(currX,currY,CELL_FREE);
        }
      }
      // Get canvas coordinates for obstacle
      int obst_x = floor(obstacle_x + canvas.cols/2);
      int obst_y = floor(obstacle_y + canvas.rows/2);
      // Check whether the coordinates are in bounds and not just returned from max sensor range
      if( range != 30.0*GRID_SCALE && inBounds(obst_x,obst_y) ) {
        // Update occupancy probability
        grid[obst_x][obst_y] = (grid[obst_x][obst_y]*smp_sig + (100*(samples-smp_sig))) / samples;
        if(grid[obst_x][obst_y] > 10)
      	  plot(obstacle_x,obstacle_y,CELL_OCCUPIED);
      }
    }
    // END MY STUFF
  };
  
  void poseCallbackNav(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = -msg->pose.pose.position.y*GRID_SCALE;
    y = msg->pose.pose.position.x*GRID_SCALE;
    // Add 90 degrees to heading because x and y were switched
    heading=tf::getYaw(msg->pose.pose.orientation) + M_PI/2.0f;
  };
  // Process incoming ground truth robot pose message
  void poseCallbackGeo(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    double roll, pitch;
    x = -msg->pose.pose.position.y*GRID_SCALE;
    y = msg->pose.pose.position.x*GRID_SCALE;
    heading=tf::getYaw(msg->pose.pose.orientation) + M_PI/2.0f;
  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    int key = 0;
    
    // Initialize all pixel values in canvas to CELL_UNKNOWN
    canvasMutex.lock();
    canvas = cv::Scalar(CELL_UNKNOWN);
    canvasMutex.unlock();
    float lastX,lastY;
    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // Plots the robot's current position
      plot(lastX,lastY,CELL_FREE);
      lastX = x;
      lastY = y;
      plot(x, y, CELL_ROBOT); // Demo code: plot robot's current position on canvas
      // NOTE: DO NOT REMOVE CODE BELOW THIS LINE
      cv::imshow("Occupancy Grid Canvas", canvas);
      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      key = cv::waitKey(1000/SPIN_RATE_HZ); // Obtain keypress from user; wait at most N milliseconds
      if (key == 'x' || key == 'X') {
        break;
      } else if (key == ' ') {
        saveSnapshot();
      }
    }
    
    ros::shutdown(); // Ensure that this ROS node shuts down properly
  };

  // Tunable motion controller parameters
  constexpr static double FORWARD_SPEED_MPS = 2.0;
  constexpr static double ROTATE_SPEED_RADPS = M_PI/2;
  
  constexpr static int SPIN_RATE_HZ = 30;
  
  constexpr static char CELL_OCCUPIED = 0;
  constexpr static char CELL_UNKNOWN = 86;
  constexpr static char CELL_FREE = 172;
  constexpr static char CELL_ROBOT = 255;

  int GRID_SCALE;

protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  ros::Subscriber poseSub; // Subscriber to the current robot's ground truth pose topic

  double x; // in simulated Stage units, + = East/right
  double y; // in simulated Stage units, + = North/up
  double heading; // in radians, 0 = East (+x dir.), pi/2 = North (+y dir.)
  
  cv::Mat canvas; // Occupancy grid canvas
  boost::mutex canvasMutex; // Mutex for occupancy grid canvas object

  std::vector<std::vector<float>> grid;
  int samples = 10;
  int smp_sig = 8;

};


int main(int argc, char **argv) {
  int width, height;
  bool printUsage = false;

  // Parse and validate input arguments
  if (argc <= 2) {
    printUsage = true;
  } else {
    try {
      width = boost::lexical_cast<int>(argv[1]);
      height = boost::lexical_cast<int>(argv[2]);

      if (width <= 0) { printUsage = true; }
      else if (height <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [CANVAS_WIDTH] [CANVAS_HEIGHT]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "grid_mapper"); // Initiate ROS node
 
  ros::NodeHandle n; // Create default handle

  char setting = 's';
  if (argc > 3) {
    if(argv[3][0] == 's' || argv[3][0] == 'o' || argv[3][0] == 'c') {
      setting = argv[3][0];
    }
  }

  int scale = 5;
  if ( argc > 4 ) {
    scale = atoi(argv[4]);
  }

  GridMapper robbie(n, width, height,setting,scale); // Create new grid mapper object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};
