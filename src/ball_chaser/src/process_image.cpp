#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) {
        ROS_ERROR("Error.");
    }
}

void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int img_center = img.step / 2 ;
    float per_bucket_turn = 0.1; //ideally should consider the distaqnce of the ball also
    int total_buckets = 6;
    int center_bucket= total_buckets / 2;
    int bucket_size = img.step / total_buckets;
    bool found_ball = false;
    int threshold = 20;
    int points_found = 0;

    int cur_column = 0;
    int cur_bucket = 0;
    float direction=0;

    int i = 0;
    bool white_pixels_found=false;
    for ( i = 0; i < img.height * img.step; ++i ) {

        if (white_pixel == img.data[i]) {
            white_pixels_found=true;
            points_found += 1;
            if(points_found > threshold){
                found_ball = true;

                cur_column = img_center - (i % img.step )   ;
                cur_bucket =  (cur_column ) / bucket_size ;
                direction = cur_bucket * per_bucket_turn;

                if(direction > 0) {
                    ROS_INFO("drive_robot: cur_column %d, cur_bucket %d, direction %f", cur_column, cur_bucket,
                             direction);
                }
                drive_robot(0.1,   direction);
                break;
            }
        } else  if(white_pixels_found){
            points_found=0;
            white_pixels_found=false;
        }
    }
    if(!found_ball) {
        drive_robot(0.0, 0.0);;
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("process_image starting");
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ROS_INFO("process_image  started spin");
    ros::spin();

    return 0;
}
