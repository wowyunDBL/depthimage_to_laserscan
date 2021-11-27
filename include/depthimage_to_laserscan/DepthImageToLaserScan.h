/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#ifndef DEPTH_IMAGE_TO_LASERSCAN
#define DEPTH_IMAGE_TO_LASERSCAN

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depthimage_to_laserscan/depth_traits.h>
#include <sstream>
#include <limits.h>
#include <math.h>

#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <time.h>

using namespace std;
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

namespace depthimage_to_laserscan
{
  struct gridValueStruct{
    int gridValue;
    int count;

    bool operator == (const int &now){
        return (this->gridValue == now);
    }
  };
  class DepthImageToLaserScan
  {
  public:
    DepthImageToLaserScan();
    ~DepthImageToLaserScan();

    /**
     * Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
     *
     * This function converts the information in the depth encoded image (UInt16 or Float32 encoding) into
     * a sensor_msgs::LaserScan as accurately as possible.  To do this, it requires the synchornized Image/CameraInfo
     * pair associated with the image.
     *
     * @param depth_msg UInt16 or Float32 encoded depth image.
     * @param info_msg CameraInfo associated with depth_msg
     * @return sensor_msgs::LaserScanPtr for the center row(s) of the depth image.
     *
     */
    sensor_msgs::LaserScanPtr convert_msg(const sensor_msgs::ImageConstPtr& depth_msg,
					   const sensor_msgs::CameraInfoConstPtr& info_msg);

    /**
     * Sets the scan time parameter.
     *
     * This function stores the desired value for scan_time.  In sensor_msgs::LaserScan, scan_time is defined as
     * "time between scans [seconds]".  This value is not easily calculated from consquetive messages, and is thus
     * left to the user to set correctly.
     *
     * @param scan_time The value to use for outgoing sensor_msgs::LaserScan.
     *
     */
    void set_scan_time(const float scan_time);

    /**
     * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
     *
     * range_min is used to determine how close of a value to allow through when multiple radii correspond to the same
     * angular increment.  range_max is used to set the output message.
     *
     * @param range_min Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
     * @param range_max Maximum range to use points in the output scan.
     *
     */
    void set_range_limits(const float range_min, const float range_max);

    /**
     * Sets the number of image rows to use in the output LaserScan.
     *
     * scan_height is the number of rows (pixels) to use in the output.  This will provide scan_height number of radii for each
     * angular increment.  The output scan will output the closest radius that is still not smaller than range_min.  This function
     * can be used to vertically compress obstacles into a single LaserScan.
     *
     * @param scan_height Number of pixels centered around the center of the image to compress into the LaserScan.
     *
     */
    void set_scan_height(const int scan_height);

    /**
     * Sets the frame_id for the output LaserScan.
     *
     * Output frame_id for the LaserScan.  Will probably NOT be the same frame_id as the depth image.
     * Example: For OpenNI cameras, this should be set to 'camera_depth_frame' while the camera uses 'camera_depth_optical_frame'.
     *
     * @param output_frame_id Frame_id to use for the output sensor_msgs::LaserScan.
     *
     */
    void set_output_frame(const std::string& output_frame_id);

    void set_show_mask(const bool& show_mask);
    void set_scan_height_limits(const int scan_height_min, const int scan_height_max);

  private:
    /**
     * Computes euclidean length of a cv::Point3d (as a ray from origin)
     *
     * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
     *
     * @param ray The ray for which the magnitude is desired.
     * @return Returns the magnitude of the ray.
     *
     */
    double magnitude_of_ray(const cv::Point3d& ray) const;

    /**
     * Computes the angle between two cv::Point3d
     *
     * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
     * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
     *
     * @param ray1 The first ray
     * @param ray2 The second ray
     * @return The angle between the two rays (in radians)
     *
     */
    double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;

    /**
     * Determines whether or not new_value should replace old_value in the LaserScan.
     *
     * Uses the values of range_min, and range_max to determine if new_value is a valid point.  Then it determines if
     * new_value is 'more ideal' (currently shorter range) than old_value.
     *
     * @param new_value The current calculated range.
     * @param old_value The current range in the output LaserScan.
     * @param range_min The minimum acceptable range for the output LaserScan.
     * @param range_max The maximum acceptable range for the output LaserScan.
     * @return If true, insert new_value into the output LaserScan.
     *
     */
    bool use_point(const float new_value, const float old_value, const float range_min, const float range_max) const;

    void writeToCSVfileDouble(string name, Eigen::MatrixXf matrix) const
    {
        ofstream file(name.c_str());
        file << matrix.format(CSVFormat);
    }

    void writeToCSVfileBool(string name, Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> matrix) const
    {
        ofstream file(name.c_str());
        file << matrix.format(CSVFormat);
    }

    double get_gridValue( vector<struct gridValueStruct> vec_gridValueStruct, bool flag ) const
    {
      int max=0, tmp, tmpV;
      double value=0;
      for (int i=0;i<vec_gridValueStruct.size();i++)
      {
        tmp = vec_gridValueStruct[i].count;
        tmpV = vec_gridValueStruct[i].gridValue;
        if (max < tmp)
        {
            max = tmp;
            value = tmpV;
        }
          
      }
      if (max == 0)  //queue length is zero means depth outside range_max_
        value = -1;

      if (max < 4 && max != 0)  //density is small ...
        value = 0;

      // if (max < 4 && max != 0) //raw data is zero
      //   value = 2;
        

      // std::ofstream inFile;
      // inFile.open("/home/ncslaber/transformed_laserScan_number-30_ros_new.csv", std::ios::out | std::ios_base::app);
      // if(!inFile)     
      //   std::cout << "Can't open file!\n";
      
      // if (flag)
      //   inFile << max ;
      // else
      //     inFile << max << ',';
      // inFile.close();

      
      // std::ofstream in2File;
      // in2File.open("/home/ncslaber/transformed_laserScan_gridValue-30_ros.csv", std::ios::out | std::ios_base::app);
      // if(!in2File)     
      // std::cout << "Can't open file!\n";
      
      // if (flag)
      //   in2File << value ;
      // else
      //     in2File << value << ',';
      // in2File.close();

      return value;
    }

    /**
    * Converts the depth image to a laserscan using the DepthTraits to assist.
    *
    * This uses a method to inverse project each pixel into a LaserScan angular increment.  This method first projects the pixel
    * forward into Cartesian coordinates, then calculates the range and angle for this point.  When multiple points coorespond to
    * a specific angular measurement, then the shortest range is used.
    *
    * @param depth_msg The UInt16 or Float32 encoded depth message.
    * @param cam_model The image_geometry camera model for this image.
    * @param scan_msg The output LaserScan.
    * @param scan_height The number of vertical pixels to feed into each angular_measurement.
    *
    */
    template<typename T>
    void convert(const sensor_msgs::ImageConstPtr& depth_msg, const image_geometry::PinholeCameraModel& cam_model,
        const sensor_msgs::LaserScanPtr& scan_msg, const int& scan_height) const{
      clock_t start, end;
      double cpu_time_used;
      start = clock();
      
      // Use correct principal point from calibration
      const float center_x = cam_model.cx();
      const float center_y = cam_model.cy();

      // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
      const double unit_scaling = depthimage_to_laserscan::DepthTraits<T>::toMeters( T(1) ); //return 0.001
      const float constant_x = unit_scaling / cam_model.fx();
      const float f_y = cam_model.fy();
      
      const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
      const int row_step = depth_msg->step / sizeof(T);

      const int offset = (int)(center_y - scan_height/2);
      // depth_row += offset*row_step; // Offset to center of image
      vector<struct gridValueStruct> vector_diffLayer[(int)depth_msg->width];
      vector<struct gridValueStruct>::iterator it;
      struct gridValueStruct gv;

      /*get depth cvMatrix*/
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);  //BGR8
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      Eigen::MatrixXf fdepth(depth_msg->height, depth_msg->width);
      cv::cv2eigen(cv_ptr->image, fdepth);
      // writeToCSVfileDouble("/home/ncslaber/fdepth-2_ros.csv", fdepth);
      // cv::namedWindow("Image window");
      // cv::imshow("Image window", cv_ptr->image);
      // cv::waitKey(0);

      /*get height mask*/
      // fdepth.array() = (fdepth.array()<15000? fdepth.array(): 0);
      // Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> maskDep = fdepth.array()<15000;
      // writeToCSVfileBool("/home/ncslaber/maskDep-2_ros.csv", maskDep);
      // fdepth = ( fdepth.array() * maskDep.array().cast<float>() ).matrix();

      double theta = 0./180.*3.1415926;
      Eigen::ArrayXf arrayPointY = Eigen::ArrayXf::LinSpaced(depth_msg->height, 0, depth_msg->height-1); 
      arrayPointY = arrayPointY - center_y;
           
      auto diaPointY( arrayPointY.matrix().asDiagonal() ); 
      
      
      Eigen::MatrixXf matPointY(diaPointY*fdepth);
      matPointY = matPointY/ (-f_y);
      matPointY = matPointY * cos(theta) + fdepth * sin(theta);
      matPointY = matPointY.array() + 410.0;      
      
      Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> mask1 = matPointY.array()<scan_height_max_; 
      Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> mask2 = matPointY.array()>scan_height_min_; 
      Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> mask3 = matPointY.array()!=410;
      Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> matMaskingHeight = ( mask1.array() * mask2.array() ).matrix();
      matMaskingHeight = ( matMaskingHeight.array() * mask3.array() ).matrix();
      
      Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> maskD = fdepth.array()<=range_max_/unit_scaling; 
      // std::cout<<"range_max_: " <<range_max_<<std::endl;
      matMaskingHeight = ( matMaskingHeight.array() * maskD.array() ).matrix();
      bool show_mask_=true;
      if (show_mask_)
      {
        cv::Mat test_image;
        cv::eigen2cv(matMaskingHeight, test_image);
        cv::threshold(test_image, test_image, 0, 255, cv::THRESH_BINARY);
        cv::imshow("mask window", test_image);
        cv::waitKey(1);
      }
      // writeToCSVfileBool("/home/ncslaber/matMaskingHeight-30_ros_new.csv", matMaskingHeight);

      // for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step)
      for(int v = 0; v < depth_msg->height; ++v, depth_row += row_step)
      {
        if ( matMaskingHeight.row(v).array().any() ) //
        {
          for (int u = 0; u < (int)depth_msg->width; ++u) // Loop over each pixel in row
          {
            if ( matMaskingHeight(v,u) == true )
            {  
              const T depth = depth_row[u];
              
              double r = depth; // Assign to pass through NaNs and Infs
              const double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
              const int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;

              if (!(range_min_/unit_scaling <= r)){
                    r = 0;
              }
              

              if (depthimage_to_laserscan::DepthTraits<T>::valid(r)){ // Not NaN or Inf in mm
                // Calculate in XYZ
                // double x = (u - center_x) * r * constant_x;
                // double z = depthimage_to_laserscan::DepthTraits<T>::toMeters(r);

                // Calculate actual distance
                // r = hypot(x, z);
                
                /*grid filter*/
                int now = (int)( (float)r/50+0.5 );
                
                it = std::find(vector_diffLayer[u].begin(), vector_diffLayer[u].end(), now);

                if ( it != vector_diffLayer[u].end() )
                {
                    it->count += 1;
                }
                else
                {
                    gv.gridValue = now;
                    gv.count = 1;
                    vector_diffLayer[u].push_back( gv );
                }

              }

              else{
                std::cout<<"it should not be printed!" << r << endl;
              }

              
              // // Determine if this point should be used.
              // if(use_point(r, scan_msg->ranges[index], scan_msg->range_min, scan_msg->range_max)){
              //   scan_msg->ranges[index] = r;
            }
          }
        }
      }
    
      double data;
      // double scan[(int)depth_msg->width];
      for (int i = 0; i < (int)depth_msg->width; ++i)
      {
        if (i==(int)depth_msg->width-1)
            data = (get_gridValue( vector_diffLayer[i], true )-1) * 0.05 + 0.025; // data in meter
        else
            data = (get_gridValue( vector_diffLayer[i], false )-1) * 0.05 + 0.025; // data in meter
        
        if (data > -0.025 && data < 0) // density is small
        {
            data = 0;
        }
        else if (data < -0.03) // outside range
        {
            data = range_max_;
        }
        double x = ( i - center_x) * data / cam_model.fx();
        data = hypot(x, data);
        

        // scan[i] = data;
        scan_msg->ranges[(int)depth_msg->width-i-1] = data;
        
      }

      /*std::ofstream newFile;
      newFile.open("/home/ncslaber/transformed_laserScan_depth-30_ros_new.csv", std::ios::out | std::ios::trunc);
      if(!newFile)     
        std::cout << "Can't open file!\n";
      else
        std::cout<<"File open successfully!\n";

      for (int i=0; i<(int)depth_msg->width;i++) {
        // std::cout << *(scan+i) << std::endl;
        newFile << *(scan+i);
        if (i!=(int)depth_msg->width-1)
        {
            newFile << ',';
        }
      }
      newFile.close();*/

      end = clock();
      cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
      std::cout << "time for convert: " << cpu_time_used <<endl; //unit: s
    }

    image_geometry::PinholeCameraModel cam_model_; ///< image_geometry helper class for managing sensor_msgs/CameraInfo messages.

    float scan_time_; ///< Stores the time between scans.
    float range_min_; ///< Stores the current minimum range to use.
    float range_max_; ///< Stores the current maximum range to use.
    int scan_height_; ///< Number of pixel rows to use when producing a laserscan from an area.
    int scan_height_min_; ///< Number of pixel rows to use when producing a laserscan from an area.
    int scan_height_max_; ///< Number of pixel rows to use when producing a laserscan from an area.
    std::string output_frame_id_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.
    bool show_mask_; ///< Output frame_id for each laserscan.  This is likely NOT the camera's frame_id.
  };

}; // depthimage_to_laserscan

#endif
