
#include <iostream>
#include <string>

#include "OpenCV.h"
#include "FAST/fast_corner.h"
#include "FAST/prototypes.h"
#include "FAST/ShiTomasi.h"
#include <experimental/filesystem>
#include <vector>
#include <string>
#include <map>
#include <chrono>

#include "handy_utils.h"
//#include "stereo_frame_wrapper.h"

#include "refractive_camera.h"
//#include "stereo_calib_optimizer.h"
//#include "particle_optimizer.h"

#include "point_correspondences.h"
#include "Quaternion.h"
#include "bundle_optimizer.h"
#include "water_refraction_calibration.h"

#include "absolute_orientation/rotation_averaging.h"


#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ceres/ceres.h> // for a hello world example
#include "air_refraction_calibration.h"

#include "bounded_variable.h"

// Good bagfile path:  
// roslaunch auv3d_system play_bag.launch bagfiles:="$(find ~/NAS1_data/projects/data/000000_02_R_n_D/underwater/SubSLAM/20181002_GwyntYMor/2018_10_04/left/ros/2018_10_04-10_11_*.bag


using namespace std::chrono;
using namespace handy_utils;
using namespace RigidTransforms;

namespace fs = std::experimental::filesystem;



std::map<std::string, std::pair<std::string, std::string>> GetStereoImageFilenames(const std::string& directory)
{
    std::map<std::string, std::pair<std::string, std::string>> left2right;
    
    for (const auto & file_entry : fs::directory_iterator(directory))
    {
        const std::string& filename = file_entry.path().filename().string();
        const std::string& filename_stem = file_entry.path().stem().string();
        
        size_t left_str_pos = filename.find("Left");
        if ( left_str_pos != std::string::npos)
        {
            // 1. replace "Left" with "Right" and see if the file exists
            const std::string prefix = filename.substr(0, left_str_pos);
            const std::string postfix = filename.substr(left_str_pos + 4);
                    
            const std::string right_filename = prefix + "Right" + postfix;
            // if the paired (right) stereo file exists, then look for camarea parameters
            // and, if found make the corresponding map entries, or simply leave the camera parameters string empty
            fs::path right_file = fs::path(directory) / fs::path(right_filename);
            fs::path left_file = fs::path(directory) / fs::path(filename);
            if (fs::exists(right_file.string()))
            {
                
                // 1. Create the entry by assigning the right file name to the left (key) and assigning empty string for parameter filename
                left2right[left_file.string()] = std::pair<std::string, std::string>(right_file.string(), "");
                
                // 2. Replace "Left" with "Rig" and see if the file exists            
                const std::string params_postfix = filename_stem.substr(left_str_pos + 4);
                const std::string params_filename = prefix + "Rig" + params_postfix + ".yaml";
                
                // 3. Try to find the parameter file and if found, make the assignment...
                fs::path params_file = fs::path(directory) / fs::path(params_filename);
                if (fs::exists(params_file.string()))
                {
                    left2right[left_file.string()].second = params_file.string();
                }
            }
        }
        
    } 
    
    return left2right;
}

//
// Save a bunch of images to a given directory with a name prefix
void SaveImages(const std::string& directory, const std::string& name_prefix, const std::vector<cv::Mat>& images, const std::string& ext = ".jpg")
{
    for (size_t i = 0; i < images.size(); i++)
    {
        const std::string filename = ( fs::path(directory) / fs::path(name_prefix + std::to_string(i)+ext) ).string();
        cv::imwrite(filename, images[i]);
    }
}

//
// Save a bunch of images to a given directory with a given names
void SaveImages(const std::string& directory, const std::vector<std::string>& names, const std::vector<cv::Mat>& images, const std::string& ext = ".jpg")
{
    
    assert( names.size() == images.size() );
    for (size_t i = 0; i < images.size(); i++)
    {
        const std::string filename = ( fs::path(directory) / fs::path( names[i] + ext) ).string();
        cv::imwrite(filename, images[i]);
    }
}

//
// Save a bunch of parameters corresponding to optimizer particles
/*void SaveParams(const std::string& directory, const std::vector<std::string>& names, const std::vector<online_calibration::OptimizerParticle>& particles)
{
    
    assert( names.size() == particles.size() );
    
    for (size_t i = 0; i < particles.size(); i++)
    {
        const std::string filename = ( fs::path(directory) / fs::path( names[i] ) ).string();
         
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
        const auto& p = particles.at(i);
        cv::Mat_<float> mK1(3, 3), mK2(3, 3), mR1(3, 3), mR2(3, 3), mpos2(3, 1), mdist1(1, 5), mdist2(1, 5); 
        const cv::Matx<float, 3, 3>& K1 = p.camera1.rbegin()->Intrinsics(), K2 = p.camera2.rbegin()->Intrinsics(), 
                                     R1 = p.camera1.rbegin()->RotationMatrix(), R2 = p.camera2.rbegin()->RotationMatrix();
        const cv::Vec<float, 5>& dist1 = p.camera1.rbegin()->DistortionCoefficients(), 
                                 dist2 = p.camera2.rbegin()->DistortionCoefficients(); 
        const cv::Vec<float, 3>& pos2 = p.camera2.rbegin()->Position();
        Kappa
        
        for (size_t r = 0; r < 3; r++)
        {.t()
            mpos2.at<float>(r, 0) = pos2[r];
            for (size_t c = 0; c < 3; c++)
            {
                mK1.at<float>(r, c) = K1(r, c);
                mK2.at<float>(r, c) = K2(r, c);
                mR1.at<float>(r, c) = R1(r, c);
                mR2.at<float>(r, c) = R2(r, c);
            }
        }
        for (size_t c = 0; c < 5; c++)
        {
            mdist1.at<float>(0, c) = dist1[c];
            mdist2.at<float>(0, c) = dist2[c];
        }
        
        size_t N = p.points.size();
        cv::Mat_<float> mXw(1, N), mYw(1, N), mZw(1, N);
        
        for (size_t c = 0; c < N; c++)
        {
            mXw.at<float>(0, c) = p.points[c].direction[0] * p.points[c].depth;
            mYw.at<float>(0, c) = p.points[c].direction[1] * p.points[c].depth;
            mZw.at<float>(0, c) = p.points[c].direction[2] * p.points[c].depth;
        }
        
        fs << "K1" << mK1;
        fs << "dist1" << mdist1;
        fs << "R1" << mR1;
        
        fs << "K2" << mK2;
        fs << "dist2" << mdist2;
        fs << "R2" << mR2;
        fs << "pos2" << mpos2;
        fs << "Xw" << mXw;
        fs << "Yw" << mYw;
        fs << "Zw" << mZw;
        
        fs.release();
    }
}

*/

struct CostFunctor 
{
   template <typename P>
   bool operator()(const P* const x, P* residual) const 
   {
     residual[0] = P(10.0) - x[0];
     return true;
   }
};


double PointDistance(size_t index, 
                     const std::vector<cv::Point2f>& points1, 
                     const std::vector<cv::Point2f>& points2)
{
    assert(index < points1.size() && points1.size() == points2.size() );
    
    double dist = sqrt( ( points1[index].x - points2[index].x ) * ( points1[index].x - points2[index].x ) +
                        ( points1[index].y - points2[index].y ) * ( points1[index].y - points2[index].y ) );
    return dist;
}

// 
// A function that works-out the average difference of two lists of points
double AvergaePointDistance(const std::vector<cv::Point2f>& points1, 
                            const std::vector<cv::Point2f>& points2)
{
    
    assert(points1.size() == points2.size() && points1.size() > 0);
    
    double avg_dist = 0.0;
    for (size_t i = 0; i < points1.size(); i++)
    {
        double dist = sqrt( ( points1[i].x - points2[i].x ) * ( points1[i].x - points2[i].x ) + 
                            ( points1[i].y - points2[i].y ) * ( points1[i].y - points2[i].y ) );
        avg_dist += dist;
    }
    
    avg_dist /= points1.size();
    
    return avg_dist;
}





int main()
{
    // Test radial distortion discrepancy between iterative and non-iterative method(s)
    cv::Matx<double, 3, 3> K_left( 1603.171084257582, 0, 1052.44234224645,
                                    0, 1608.6015952997, 558.0086374766669,
                                    0, 0, 1);
    cv::Vec<double, 5> dist_left(0.0258113, 0.501055, 0.00129714, -0.000607445, 0);
    
    online_calibration::PinholeCamera<5, double>camera_left(cv::Size_<int>(800, 600), K_left, dist_left);
    
    online_calibration::RefractiveCamera<5, double> rcam (camera_left, 
                                                        cv::Vec<double, 3>(0.00435152, 0.0177847, 0.999832),
                                                        0.00600,
                                                        0.010,
                                                        1.000293/1.458,
                                                        1.458/1.333);
    
    std::cout << " Normal : " << rcam.Normal() << std::endl;
    cv::Vec<double, 2> p(23, 24);
    cv::Vec<double, 3> v0 = rcam.V0( p );
    std::cout << " V0 : " << v0 << std::endl;
    cv::Matx<double, 3, 2> U = rcam.RefractionBasis(p);
    cv::Vec<double, 3> v1 = rcam.V1( p , U);
    std::cout << " Refraction plane basis : " << U << std::endl;
    std::cout << " V1 from image projection: " << v1 << std::endl;
    cv::Vec<double, 3> v1fromv0 = rcam.V1fromV0( v0 , U);
    std::cout << " V1 from V0: " << v1fromv0 << std::endl;
    
    
    cv::Vec<double, 3> v2 = rcam.V2( p , U);
    std::cout << " V2 from image projection: " << v2 << std::endl;
    cv::Vec<double, 3> v2fromv1 = rcam.V2fromV1( v1 , U);
    
    std::cout << " V2 from V1: " << v2fromv1 << std::endl;
    
    online_calibration::RefractionTrajectory<double> trajectory = rcam.BackProject( p, 29.67 );
    
    std::cout << "\nBack-projected trajectory:  " << trajectory << std::endl;
    
    online_calibration::RefractionTrajectory<double> trajectory2 = rcam.BackProject( p, trajectory.M);
    
    std::cout << "\nBack-projected trajectory using the back-projected point:  " << trajectory2 << std::endl;
    
    
    // project back to image and see what we get.....
    auto start = high_resolution_clock::now(); 
    
    online_calibration::RefractionTrajectory<double> trajectory1 = rcam.Project( trajectory.M );
    
    auto stop = high_resolution_clock::now(); 
    auto duration = duration_cast<microseconds>(stop - start); 
    std::cout << " Projection time in miscroseconds : " << duration.count() << std::endl;
    
    std::cout << "\nProjected trajectory:  " << trajectory1 << std::endl;
    
   // exit(0);
    
    
    /*double phi1 = 30 * M_PI / 180;
    double phi2 = 37 * M_PI / 180;
    
    cv::Matx<double, 3, 3> Rr1(0.9868560346016421, 0.1255134603679525, -0.1017916412913459, 
                               -0.12674959880252, 0.9919181235434732, -0.005742420168895015,
                                0.1002482227958374, 0.01856899169202034, 0.9947891667955672);
    
    cv::Matx<double, 3, 3> Rr2(0.9898589024824103, 0.1006374408740561, -0.1002569632027574,
                               -0.1024596252930527, 0.9946498214600068, -0.01318172425685271,
                                0.09839399555422802, 0.02332033798848788, 0.9948742551071352);
    
    absolute_orientation::RotationAlignmentAboutAxis<double> alignment({Rr1, Rr2});
    
    alignment.Initialize();
    alignment.Solve();
    std::cout << " Rotation R1 " << Rr1 << std::endl;
    std::cout << " Rotation R2 " << Rr2 << std::endl;
    std::cout << " Aligning Rotation  R " << alignment.R() << std::endl;
    std::cout << " New Rotation  R1 " << *alignment.NewFrame(0) << std::endl;
    std::cout << " New Rotation  R2 " << *alignment.NewFrame(1) << std::endl;
    
    exit(0);
    */
    /*
    online_calibration::LogisticVariable<double> v(0.568341, 0.574885, 10);
    std::cout << " Logistic Variable : " << v << std::endl;
    std::cout << " Weight : " << v.W();
    std::cout << " Constant : " << v.C() << std::endl;;
    std::cout << " Max value : " << v.ValueMax();
    std::cout << " Min value : " << v.ValueMin() << std::endl;;
    
    double val = 0.571596;

    std::cout << " Computed parameter for value " << val << ": " << v.Parameter(val) << std::endl;
    
    double par = 0;
    std::cout << " Computed parametr for[0.9838737748265902, 0.104631466037246, -0.1450677480468121;
 -0.1055378551959449, 0.9944142214233382, 0.001455112248698561;
 0.1444096822554377, 0.01387849220604281, 0.9894206542846025] from initial R1: [0.9999854481333973, 0.005393979130469385, 9.225285215875976e-05;
 -0.005394367708090265, 0.9999733496852427, 0.004919422354847875;
 -6.571513207519848e-05, -0.004919848413876668, 0.9999878953132914] value" << val << " is: " << v.Value(v.Parameter(val)) << std::endl;
    std::cout << " Computed value for t = 10 : " << v.Value(10) << std::endl;
    std::cout << " Computed value for t = -10 : " << v.Value(-10) << std::endl;
    
    exit(0);
    */
    /*cv::Matx<double, 3, 3> K(750, 0, 250, 0, 800, 300, 0, 0, 1);
    cv::Mat_<double> mK = cv::Mat_<double>(K);
    std::cout << " cv::Matx<double, 3, 3> K : " << mK << std::endl;
    std::cout << " cv::Mat mK from K : " << mK << std::endl;
    
    cv::Mat mt(3, 1, CV_32F);
    mt.at<float>(0, 0) = 1.1; mt.at<float>(1, 0) = 2.2; mt.at<float>(2, 0) = 3.3;
    cv::Vec<double, 3> t = mt;
    std::cout << " cv::Mat float vector mt: " << mt << std::endl;
    std::cout << " cv::Vec<double, 3> t = mt: " << t << std::endl; 
    
    cv::Mat mR(2, 2, CV_64F);
    mR.at<double>(0, 0) = 1.1; mR.at<double>(0, 1) = 1.2;
    mR.at<double>(1, 0) = 2.1; mR.at<double>(1, 1) = 2.2;
    
    cv::Matx<float, 2, 2> R = mR;
    std::cout << " cv::Mat (double) mR : " << mR << std::endl;
    std::cout << "cv::Matx<float, 2,2>  R = mR : " << R << std::endl;
    exit(0);
    */
    //cv::VideoCapture cap("/home/george/georges_sandbox/chessboard_test_sequence/Chessboard9x7.mp4");
    cv::VideoCapture left_air2air_cap("/media/george/EF9D-64AC/videos2020_01_23/left12_29.mp4");
    cv::VideoCapture right_air2air_cap("/media/george/EF9D-64AC/videos2020_01_23/right12_29.mp4");
    
    //cv::VideoCapture air2glass_cap("/media/george/EF9D-64AC/videos2020_01_23/left12_52.mp4");
    cv::VideoCapture left_air2glass_cap("/media/george/EF9D-64AC/videos2020_01_23/left12_42.mp4");
    cv::VideoCapture right_air2glass_cap("/media/george/EF9D-64AC/videos2020_01_23/right12_42.mp4");
    
    // The underwater stereo 
    cv::VideoCapture left_water_cap("/media/george/EF9D-64AC/videos2020_01_23/left14_24.mp4");
    cv::VideoCapture right_water_cap("/media/george/EF9D-64AC/videos2020_01_23/right14_24.mp4");
    
    
    // 
    // 1. Camera calibration in air (no glass)
    int num_air2air_images = 50;
    int num_air2glass_images = 50;
    double corner_dist_threshold = 160;
    
    std::vector<std::vector<cv::Point2f>> air2air_corners;
    std::vector<std::vector<cv::Point2f>> left_air2glass_corners;
    std::vector<std::vector<cv::Point2f>> right_air2glass_corners;
    
    const cv::Size_<int> board_size = cv::Size_<int>(6, 4);
    
    // Create a vector with the chessboard points in a local frame
    std::vector<std::vector<cv::Point3f>> object_points;
    object_points.push_back(std::vector<cv::Point3f>());
    float square_dim = 0.071;
    for (int r = 0; r < board_size.height; r++)
    {
        for (int c = 0; c < board_size.width; c++)
        {
            float x = c * square_dim, y = r * square_dim;
            object_points[0].push_back(cv::Point3f(x, y, 0));
        }
    
    }
    
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0, 1);
    
    cv::Size_<int> left_image_size, right_image_size;
    int num_registered_images = 0;
    int skip_counter = 0;
    
    std::vector<cv::Point2f> previous_corners;
    /*
    while (num_registered_images < num_air2air_images)
    {
        air2air_cap.grab();
        cv::Mat image;
        
        air2air_cap.retrieve(image);
        image_size = image.size();
        cv::Mat_<uchar> gray_image(image.rows, image.cols);
        cv::Mat_<cv::Vec3b> rgb_image(image.rows, image.cols);
        image.copyTo(rgb_image);
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
        //if ( dist(gen) < 0.9 )
        //{
        //    std::cout << " Skipping this one! "<< std::endl;
        //    continue;
        //}        
        std::vector<cv::Point2f> corners;
        bool patternfound = cv::findChessboardCorners(gray_image, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        
        if(patternfound)
        {
            if(num_registered_images == 0)
            {
                previous_corners = corners;
            }
            else 
            {
                //if ( AvergaePointDistance(previous_corners, corners) < corner_dist_threshold ) continue;
                if ( PointDistance(12, previous_corners, corners) < corner_dist_threshold ) continue;
                
                previous_corners = corners;
            }
                
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
            
            air2air_corners.push_back(corners);
            num_registered_images++;
            cv::drawChessboardCorners(rgb_image, board_size, corners, patternfound);
            
        }
        else
        {
            std::cerr << "\n Could not find any corners!!!\n";
        }
        
        cv::namedWindow("capture");
        cv::imshow("capture", rgb_image);
        cv::waitKey(5);
    }
    
    //
    // Filling-in the remaining object points
    for (int i = 1; i < air2air_corners.size(); i++)
    {
        object_points.push_back(object_points[0]);
    }
    
    //
    std::cout << "\n\t Number of points on the chessbard : " << board_size.width << " x " << board_size.height << " x " << air2air_corners.size() << " = " << object_points[0].size() << " x " << air2air_corners.size() << std::endl;
    std::cout << "\n\tNumber of chessboards captured : " << air2air_corners.size()<<std::endl;
    std::cout << "\n\t Calibrating ...\n";
    
    
    std::vector<cv::Mat> Rvecs, tvecs;
    cv::Mat mK, mdist;
    cv::calibrateCamera(object_points, air2air_corners, image_size, mK, mdist, Rvecs, tvecs );
    
    std::cout << "\n\tCalibrated camera intrinsics : " << mK << std::endl;
    std::cout << "\n\tCalibrated camera distortion coefficients : " << mdist << std::endl;
    */
    //
    // Left-Right Camera parameters at 12:38
    cv::Matx<double, 3, 3> leftxK(1187.379029142634, 0, 1061.129526339368, 0, 1192.254178283229, 554.0148828507155, 0, 0, 1);
    cv::Mat leftK(leftxK);
    cv::Vec<double, 5> leftxdist(-0.1648973735422724, 0.0902462645598357, 4.03941533135392e-05, 0.0005921674614293508, 0);
    cv::Mat leftdist( leftxdist );
    cv::Matx<double, 3, 3> leftxR( 0.9999531850860439, 0.008581969465889835, 0.004469612551762656, 
                                            -0.008583939431338759, 0.9999630683737879, 0.0004217491180030002, 
                                            -0.00446582804364961, -0.0004600962572802063, 0.9999899222948793  );
    cv::Mat leftR( leftxR );
    cv::Matx<double, 3, 4> leftxP( 1156.516516340331, 0, 1035.884460449219, 0, 0, 1156.516516340331, 552.9626693725586, 0, 0, 0, 1, 0 );
    cv::Mat leftP( leftxP );
    left_image_size = cv::Size_<int>(2048, 1080);
    double left_global_scaler_x = 0.5;
    double left_global_scaler_y = 0.5;
    
    cv::Matx<double, 3, 3> rightxK( 1184.637420718189, 0, 1031.000186588863, 0, 1188.983890474427, 552.3985978992571, 0, 0, 1 );
    cv::Mat rightK( rightxK );
    cv::Vec<double, 5> rightxdist(-0.1656677695366082, 0.08781520451532637, -0.0009571089397685708, 0.0002839024076617939, 0);
    cv::Mat rightdist( rightxdist );
    cv::Matx<double, 3, 3> rightxR(0.9999767433207524, -0.002256281710643894, 0.00643599335490774, 
                                            0.00225900836254522, 0.9999973617348911, -0.0004164186303409823, 
                                            -0.006435036817311415, 0.0004309479086364913, 0.9999792020762531);
    cv::Mat rightR( rightxR );
    cv::Matx<double, 3, 4> rightxP(1156.516516340331, 0, 1035.884460449219, -139.9439850549975, 0, 1156.516516340331, 552.9626693725586, 0, 0, 0, 1, 0 );
    cv::Mat rightP( rightxP );
    right_image_size = cv::Size_<int>(2048, 1080);
    double right_global_scaler_x = 0.5;
    double right_global_scaler_y = 0.5;
    
    // Create camera objects (we are not worried about the stereo stuff here)
    online_calibration::PinholeCamera<5, double> left_camera(left_image_size, 
                                                            leftxK,
                                                            leftxdist,
                                                            leftxR,
                                                            cv::Vec<double, 3>(0, 0, 0),
                                                            left_global_scaler_x,
                                                            left_global_scaler_y
                                                           );
    
    online_calibration::PinholeCamera<5, double> right_camera(right_image_size, 
                                                             rightxK,
                                                             rightxdist,
                                                             rightxR,
                                                             cv::Vec<double, 3>(-rightxP(0, 3) / rightxP(0, 0), 0, 0), 
                                                             right_global_scaler_x,
                                                             right_global_scaler_y
                                                            );
                                                            
    
    
    //
    // 3. Now detect chessboard corners in the air2glass sequence 
    
    //
    // Calibrate left
    //
    std::cout << "\n\t ==================================== Calibrating LEFT AIR-TO-GLASS parameters. ==================================== \n";
    
    /*
    num_registered_images = 0;
    previous_corners.clear();
    skip_counter = 0;
    while (num_registered_images < num_air2glass_images)
    {
        left_air2glass_cap.grab();
        cv::Mat image;
        skip_counter++;
        if (skip_counter < 1000) continue;
        
        left_air2glass_cap.retrieve(image);
        left_image_size = image.size();
        cv::Mat_<uchar> gray_image(image.rows, image.cols);
        cv::Mat_<cv::Vec3b> rgb_image(image.rows, image.cols);
        image.copyTo(rgb_image);
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
        
        std::vector<cv::Point2f> corners;
        bool patternfound = cv::findChessboardCorners(gray_image, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        
        if(patternfound)
        {
            if (num_registered_images == 0)
            {
                previous_corners = corners;
            }
            else 
            {
                //if ( AvergaePointDistance(previous_corners, corners) < corner_dist_threshold ) continue;
                if ( PointDistance(12, previous_corners, corners) < corner_dist_threshold ) continue;
                
                previous_corners = corners;
            }
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
            
            left_air2glass_corners.push_back(corners);
            num_registered_images++;
            cv::drawChessboardCorners(rgb_image, board_size, corners, patternfound);
            
        }
        else
        {
            std::cerr << "\n Could not find any corners!!!\n";
        }
        
        cv::namedWindow("capture");
        cv::imshow("capture", rgb_image);
        cv::waitKey(5);
        std::cout <<num_registered_images<<"\r";
    }
    
    
    cv::Mat I3 = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat zero = cv::Mat::zeros(3, 1, CV_32F);
    online_calibration::PinholeCamera<5, double> left_camera(left_image_size, leftK, leftdist, I3, zero);
    online_calibration::AirGlassRefractionCalibrator left_a2gcalibrator(left_camera, 
                                                                       board_size, 
                                                                        0.071, 
                                                                        left_air2glass_corners, 
                                                                        online_calibration::RefractiveIndices::Material::SAPPHIRE_GLASS,
                                                                        0.00001,
                                                                       0.001,
                                                                        1000,
                                                                        0.010,
                                                                        0.012,
                                                                        100,
                                                                        50.0,
                                                                        8,
                                                                        -1
                                                                        );
    left_a2gcalibrator.InitializeGlassNormalAndPoses();
    left_a2gcalibrator.RunGlassNormalAndPoseOptimization();
    left_a2gcalibrator.InitializeRefractionParameters();
    left_a2gcalibrator.RunRefractionParameterOptimization();
    
    
    cv::destroyWindow("capture"); 
    */
    
    //
    // Calibrate right
    //
    std::cout << "\n\t ==================================== Calibrating RIGHT AIR-TO-GLASS parameters. ==================================== \n";
    /*
    num_registered_images = 0;
    previous_corners.clear();
    skip_counter = 0;
    while (num_registered_images < num_air2glass_images)
    {
        right_air2glass_cap.grab();
        cv::Mat image;
        skip_counter++;
        if (skip_counter < 1000) continue;
        
        right_air2glass_cap.retrieve(image);
        left_image_size = image.size();
        cv::Mat_<uchar> gray_image(image.rows, image.cols);
        cv::Mat_<cv::Vec3b> rgb_image(image.rows, image.cols);
        image.copyTo(rgb_image);
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
        
        std::vector<cv::Point2f> corners;
        bool patternfound = cv::findChessboardCorners(gray_image, board_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        
        if(patternfound)
        {
            if (num_registered_images == 0)
            {
                previous_corners = corners;
            }
            else 
            {
                //if ( AvergaePointDistance(previous_corners, corners) < corner_dist_threshold ) continue;
                if ( PointDistance(12, previous_corners, corners) < corner_dist_threshold ) continue;
                
                previous_corners = corners;
            }
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
            
            right_air2glass_corners.push_back(corners);
            num_registered_images++;
            cv::drawChessboardCorners(rgb_image, board_size, corners, patternfound);
            
        }
        else
        {
            std::cerr << "\n Could not find any corners!!!\n";
        }
        
        cv::namedWindow("capture");
        cv::imshow("capture", rgb_image);
        cv::waitKey(5);
        std::cout <<num_registered_images<<"\r";
    }
    
    
    online_calibration::PinholeCamera<5, double> right_camera(right_image_size, rightK, rightdist, I3, zero);
    online_calibration::AirGlassRefractionCalibrator right_a2gcalibrator(right_camera, 
                                                                       board_size, 
                                                                        0.071, 
                                                                        right_air2glass_corners, 
                                                                        online_calibration::RefractiveIndices::Material::SAPPHIRE_GLASS,
                                                                        0.00001,
                                                                        0.001,
                                                                        1000,
                                                                        0.010,
                                                                        0.012,
                                                                        100,
                                                                        50.0,
                                                                        8,
                                                                        -1
                                                                        );
    right_a2gcalibrator.InitializeGlassNormalAndPoses();
    right_a2gcalibrator.RunGlassNormalAndPoseOptimization();
    right_a2gcalibrator.InitializeRefractionParameters();
    right_a2gcalibrator.RunRefractionParameterOptimization();

    cv::destroyWindow("capture");
    
        
        
        
    // Retrieve normals
    cv::Vec<double, 3> left_normal = left_a2gcalibrator.Normal();
    cv::Vec<double, 3> right_normal = right_a2gcalibrator.Normal();
    
    // retrieve refractive ratios
    double left_air2glass_ratio = left_a2gcalibrator.AirToGlassRatio();
    double right_air2glass_ratio = right_a2gcalibrator.AirToGlassRatio();
    
    double left_d0 = left_a2gcalibrator.Distance0();
    double left_d1 = left_a2gcalibrator.Distance1();
    
    double right_d0 = left_a2gcalibrator.Distance0();
    double right_d1 = left_a2gcalibrator.Distance1();
    
    */
    cv::Vec<double, 3> left_normal(0.0026554, -0.0032301, 0.9999910);
    cv::Vec<double, 3> right_normal(0.010450, -0.017971, 0.999784);
    //cv::Vec<double, 3> left_normal(0.0, -0.0, 1);
    //cv::Vec<double, 3> right_normal(0.0, -0.0, 1);
    
    
    // retrieve refractive ratios
    double left_air2glass_ratio = 0.57488;
    double right_air2glass_ratio = 0.57488;
    
    double left_d0 = 2.7138e-04;
    double left_d1 = 0.010000;
    
    double right_d0 = 1.0638e-05;
    double right_d1 = 0.01;
    
        
    //
    // Create a list of stereo pairs from an underwater sequence (ANY GOOD sequence over an underwater structure)
    num_registered_images = 0;
    skip_counter = 0;
    int num_skipped_water_images = 3;
    // The vector to cache the frames
    std::vector<std::shared_ptr<online_calibration::StereoFrameWrapper>> image_pairs; // use this to store images for tracking
    int num_glass2water_images = 10;
    // Go...
    // Get list of stereo image files
    std::map<std::string, std::pair<std::string, std::string>> stereo_pair_filenames = GetStereoImageFilenames("/home/george/catkin_ws/src/online_calibration/export");
    for (const auto& pair_entry: stereo_pair_filenames)
    {
        if (skip_counter < num_skipped_water_images)
        {
            skip_counter++;
            continue;
        }
        const std::string& left_filename = pair_entry.first;
        const std::string& right_filename = pair_entry.second.first;
        const std::string& params_filename = pair_entry.second.second;
        
        cv::Mat_<cv::Vec3b> left_image = cv::imread(left_filename, cv::IMREAD_COLOR);
        cv::Mat_<cv::Vec3b> right_image = cv::imread(right_filename, cv::IMREAD_COLOR);
    /*while (num_registered_images < num_air2glass_images)
    {
        left_water_cap.grab();
        right_water_cap.grab();
        
        cv::Mat left_image, right_image;
        skip_counter++;
        if (skip_counter < num_skipped_water_images) continue;
        
        left_water_cap.retrieve(left_image);
        right_water_cap.retrieve(right_image);
        */    
        left_image_size = left_image.size();
        right_image_size = left_image.size();
    
        cv::Mat_<uchar> left_gray_image(left_image.rows, left_image.cols);
        cv::Mat_<uchar> right_gray_image(right_image.rows, right_image.cols);
        
        cv::Mat_<cv::Vec3b> left_rgb_image(left_image.rows, left_image.cols);
        cv::Mat_<cv::Vec3b> right_rgb_image(right_image.rows, right_image.cols);
        
        left_image.copyTo(left_rgb_image);
        right_image.copyTo(right_rgb_image);
        
        cv::cvtColor(left_image, left_gray_image, CV_BGR2GRAY);
        cv::cvtColor(right_image, right_gray_image, CV_BGR2GRAY);
        
        // Create the stereoframe wrapper to proc ess images and find correspondences
        std::shared_ptr<online_calibration::StereoFrameWrapper> pstereo_pair( new  online_calibration::StereoFrameWrapper(left_gray_image, 
                                                                                right_gray_image,            
                                                                                left_camera,
                                                                                right_camera
                                                                                )
                                                                           );
                                                                                    
                                                                                
        // Now build necessary pyramid images, detect features and determine image windows to be matched..
        pstereo_pair->MakeImagePyramid();
        // Create all level gradients from level PyramidSize()-1 to the top using the SE2 registration at the top level ONLY (works ok).
        pstereo_pair->MakeSE2Pyramid();
        
        // Finally, detect features at the designated levels (i.e., from base level to 0 1)
        int feature_detection_level = 0;
        pstereo_pair->DetectAndMatchFASTFeatures(feature_detection_level);
         
        image_pairs.push_back(pstereo_pair);
        
        cv::Mat matches_image;
        pstereo_pair->DrawMatches(feature_detection_level, matches_image);
        cv::namedWindow("matches");
        cv::imshow("matches", matches_image);
        
        num_registered_images++;
         cv::waitKey(-1);
         std::cout <<num_registered_images<<"\r";
         if (num_registered_images > num_glass2water_images) break;
    }
    
    online_calibration::PointCorrespondences correspondences(image_pairs);
    
    
    //
    // 
    // Now setup the water calibration...
    using namespace online_calibration;
    
    GlassWaterRefractionCalibrator g2wcalibrator( correspondences, 
                                                  left_normal,
                                                  right_normal, 
                                                  left_d0,
                                                  left_d1,
                                                  right_d0,
                                                  right_d1,
                                                  left_air2glass_ratio,
                                                  right_air2glass_ratio,
                                                  RefractiveIndices::MinRatioFromAToB(RefractiveIndices::Material::SAPPHIRE_GLASS, 
                                                                                      RefractiveIndices::Material::SEA_WATER
                                                                                      ),
                                                  RefractiveIndices::MaxRatioFromAToB(RefractiveIndices::Material::SAPPHIRE_GLASS, 
                                                                                      RefractiveIndices::Material::SEA_WATER
                                                                                      ),
                                                  RefractiveIndices::MinRatioFromAToB(RefractiveIndices::Material::SAPPHIRE_GLASS, 
                                                                                      RefractiveIndices::Material::SEA_WATER
                                                                                      ),
                                                  RefractiveIndices::MaxRatioFromAToB(RefractiveIndices::Material::SAPPHIRE_GLASS, 
                                                                                      RefractiveIndices::Material::SEA_WATER
                                                                                      ),
                                                  -1
                                                );
    
    
    g2wcalibrator.InitializeOptimization();
    g2wcalibrator.RunOptimization();
    
    exit(0);
    
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*
    cv::Matx<double, 3, 2> M(1, 2, 3, 4, 5, 6);
    //cv::Matx<double, 3, 3> U;
    cv::Mat U;
    //cv::Matx<double, 2, 2> Vt;
    cv::Mat Vt;
    //cv::Vec<double, 2> s;
    cv::Mat s;
    cv::SVD::compute(cv::Mat(M), s, U, Vt);
    
    std::cout << " The matrix : " << M << std::endl;
    std::cout << " Matrix U : " << U << std::endl;
    std::cout << " Matrix Vt : " << Vt << std::endl;
    std::cout << " Singular values in s : " << s << std::endl;
    
    exit(0);
    */
    struct passwd *pw = getpwuid(getuid());

    const char *homedir = pw->pw_dir;
    std::string home_dir(homedir);
    
    //
    // path to the data images 
    std::string calibration_data_path = home_dir + "//georges_sandbox//calibration_data";
    
  
    // ************************** Rig Parameters measured in air *************************

    //
    // ROS bag file: ~/NAS/NAS1_data/projects/data/190623_TotalFranklin/SubSLAM/2019_06_23/Left/ros/2019_06_23-10_49_0.bag
            
    cv::Matx<float, 3, 3> K1_air;
    K1_air(0, 0) = 1193.486583336385f; K1_air(0, 1) = 0;                      K1_air(0, 2) = 1029.426812688722f; 
    K1_air(1, 0) = 0;                  K1_air(1, 1) = 1197.134742820872f;     K1_air(1, 2) = 553.7245392761045f;
    K1_air(2, 0) = 0;                  K1_air(2, 1) = 0;                      K1_air(2, 2) = 1;
            
            
            
    cv::Matx<float, 3, 3> R1_air;
    R1_air(0, 0) = 0.9998631779149925f;    R1_air(0, 1) = -0.007171288259829226f; R1_air(0, 2) = 0.01490630988629994f;
    R1_air(1, 0) = 0.00719837310646537f;   R1_air(1, 1) = 0.9999725352294749f;    R1_air(1, 2) = -0.001764146070091779f; 
    R1_air(2, 0) = -0.01489324928791846f;  R1_air(2, 1) = 0.001871205876150387f;  R1_air(2, 2) = 0.9998873385108029f;
            
    cv::Vec<float, 5> dist1_air( -0.1611652381517163f, 0.09242885214091862f, -0.0008199100682291765f, -0.0001430913322622016f, 0.0f);
    cv::Vec<float, 3> t1_air(0, 0, 0);
            
            
    cv::Matx<float, 3, 3> K2_air;
    K2_air(0, 0) = 1195.896549426772f;      K2_air(0, 1) = 0;                   K2_air(0, 2) = 1025.720303618646f;
    K2_air(1, 0) = 0;                       K2_air(1, 1) = 1199.159302743522f;  K2_air(1, 2) = 575.3306858935954f;  
    K2_air(2, 0) = 0;                       K2_air(2, 1) = 0;                   K2_air(2, 2) = 1;
            
            
            
    cv::Matx<float, 3, 3> R2_air;
    R2_air(0, 0) = 0.9997589800713189f;     R2_air(0, 1) = -0.0040143878851004f;   R2_air(0, 2) = 0.02158393978549576f;
    R2_air(1, 0) = 0.003975144937133661f;   R2_air(1, 1) = 0.9999903678650358f;    R2_air(1, 2) = 0.001860752503445167f; 
    R2_air(2, 0) = -0.02159120166838169f;   R2_air(2, 1) = -0.001774504736047786f; R2_air(2, 2) = 0.9997653080315685f;
            
            
    cv::Vec<float, 5> dist2_air( -0.1630044914715142f, 0.09036027897612192f, 0.0004557560730956954f, 0.0003538629615793735f, 0 );
    cv::Vec<float, 3> t2_air( -143.9008275194472f, 0, 0);

    
    
    
    // Get list of stereo image files
    std::map<std::string, std::pair<std::string, std::string>> stereo_pairs = GetStereoImageFilenames(calibration_data_path);
    
    
    // Create an particle filter optimizer at level 1 for a pair of stereo images (see default constructor parameters in 
    // (level - 1 means that the feature detector runs at half-sampled image)
    //online_calibration::OptimizerParticleFilter pf;
    
    // Camera parmeter cache for the parameters recorded with the current stereo pair
    cv::Matx<float, 3, 3> K1, K2, R1, R2;
    cv::Matx<float, 3, 4> P1, P2;
    cv::Vec<float, 3> t1, t2;
    cv::Vec<float, 5> dist1, dist2;
    cv::Size_<int> image_size1, image_size2;
    
    // Odometry quaternion and position. 
    cv::Vec<float, 4> odom_q;
    cv::Vec<float, 3> odom_pos; // Position expressed in world frame.
    
    // Go through images
    //
    // The Left-Right rig matches
    // projection image names
    std::vector<std::string> lrpair_image_names;
    std::vector<cv::Mat> lrpair_images;
    
    //
    // The Left-Right crosspair
    std::vector<std::string> lrxpair_image_names;
    std::vector<cv::Mat> lrxpair_images;
    
    //
    // The Left-Left crosspair matches
    std::vector<std::string> llxpair_image_names;
    std::vector<cv::Mat> llxpair_images;
    
    //
    // The Right-Right crosspair matches
    std::vector<std::string> rrxpair_image_names;
    std::vector<cv::Mat> rrxpair_images;
    // 
    // Right-Left xrosspair
    std::vector<std::string> rlxpair_image_names;
    std::vector<cv::Mat> rlxpair_images;
    
    
    size_t frame_count = 0;
    const size_t n_frames_to_optimize = 3;
    const size_t start_image_index = 0;
    size_t i = 0;
    for (const auto& pair_entry: stereo_pairs)
    {
        if (i < start_image_index)
        {
            i++;
            continue;
        }
        const std::string& left_filename = pair_entry.first;
        const std::string& right_filename = pair_entry.second.first;
        const std::string& params_filename = pair_entry.second.second;
        
        
        
        // load parameters from file. 
        handy_utils::LoadStereoParamsFromFile(params_filename, image_size1, K1, dist1, R1, t1, P1, image_size2, K2, dist2, R2, t2, P2, odom_q, odom_pos);
        //K1 = K1_air;
        //K2 = K2_air;
        
        //R1 = R1_air;
        //R1 = cv::Matx<float, 3, 3>::eye();
        //R2 = R2_air;
        //R2 = cv::Matx<float, 3, 3>::eye();
        
        //dist1 = dist1_air;
        //dist2 = dist2_air;
        //t1 = t1_air;
        //t2 = t2_air;
        
        cv::Vec<float, 3> pos1(0, 0, 0);
        cv::Vec<float, 3> pos2(0.13, 0, 0);
        
        // Create quaternion object from vector form
        Quaternion<> odom_quat(odom_q[0], odom_q[1], odom_q[2], odom_q[3]);
        // Get the rotation matrix (Orientation vectors stored in column major fashion)
        cv::Matx<float, 3, 3> odom_R;
        odom_quat.RotationMatrix(odom_R.val);
        
        // Read unrectified images
        cv::Mat_<uchar> left_img = cv::imread(left_filename, cv::IMREAD_GRAYSCALE);
        cv::Mat_<uchar> right_img = cv::imread(right_filename, cv::IMREAD_GRAYSCALE);
        
        // Register the pair in the optimizer
       /* std::cout << " Registering stereo pair ... \n";
        pf.RegisterStereoPair(left_img, 
                                     right_img, 
                                     K1, dist1, R1, t1, 
                                     K2, dist2, R2, t2,
                                     odom_R, odom_pos
                                    );
        
        
        */
       std::cout <<"K1 read: " << K1 << std::endl;
       std::cout <<"R1 read: " << R1 << std::endl;
       std::cout <<"K2 read: " << K2 << std::endl;
       std::cout <<"R2 read: " << R2 << std::endl;
       std::cout <<"pos1 (fixed): " << pos1 << std::endl;
       std::cout <<"pos2 (fixed): " << pos2 << std::endl;
       cv::Matx<float, 3, 4> P2(1, 0, 0, -0.16, 0, 1, 0, 0, 0, 0, 1, 0);
        frame_count++;
    
        // ****************************** Some ad hoc stuff here **************************
        //
        std::shared_ptr<online_calibration::StereoFrameWrapper> pstereo_pair( new online_calibration::StereoFrameWrapper(
                                                                                left_img, 
                                                                                right_img,            
                                                                                online_calibration::PinholeCamera<5, double>(cv::Size_<int>( left_img.size().width * 2, left_img.size().height*2),  
                                                                                                         K1, 
                                                                                                         dist1,
                                                                                                          R1, cv::Vec<double, 3>(0, 0, 0)
                                                                                                                           ),

                                                                                online_calibration::PinholeCamera<5, double>(
                                                                                    cv::Size_<int>( right_img.size().width * 2, right_img.size().height*2),  
                                                                                                          K2,
                                                                                                          dist2,
                                                                                                          R2, 
                                                                                                          cv::Vec<double, 3>(-P2(0, 3)/P2(0, 0), 0, 0)
                                                                                                   ),
                                                                                odom_R,
                                                                                odom_pos
                                                                                )
                                                                );
                                                                                    
                                                                                
        
         // Now build necessary pyramid images, detect features and determine image windows to be matched..
         pstereo_pair->MakeImagePyramid();
         // Create all level gradients from level PyramidSize()-1 to the top using the SE2 registration at the top level ONLY (works ok).
         pstereo_pair->MakeSE2Pyramid();
        
         // Finally, detect features at the designated levels (i.e., from base level to 0 1)
         int feature_detection_level = 0;
         pstereo_pair->DetectAndMatchFASTFeatures(feature_detection_level);
         
         image_pairs.push_back(pstereo_pair);
         
         
         if (image_pairs.size() > 1)
         {
             /*pstereo_pair->MatchCrossPairFeatures(feature_detection_level,  *(image_pairs[0]), 0);
             cv::Mat xpair_ll_matches_canvas, xpair_lr_matches_canvas, xpair_rl_matches_canvas, xpair_rr_matches_canvas;
             
             pstereo_pair->DrawLeftLeftCrosspairMatches(feature_detection_level, image_pairs, 0, xpair_ll_matches_canvas);
             cv::namedWindow("left-left matches");
             cv::imshow("left-left matches", xpair_ll_matches_canvas);
             
             pstereo_pair->DrawLeftRightCrosspairMatches(feature_detection_level, image_pairs, 0, xpair_lr_matches_canvas);
             cv::namedWindow("left-right matches");
             cv::imshow("left-right matches", xpair_lr_matches_canvas);
             
             pstereo_pair->DrawRightLeftCrosspairMatches(feature_detection_level, image_pairs, 0, xpair_rl_matches_canvas);
             cv::namedWindow("right-left matches");
             cv::imshow("right-left matches", xpair_rl_matches_canvas);
             
             pstereo_pair->DrawRightRightCrosspairMatches(feature_detection_level, image_pairs, 0, xpair_rr_matches_canvas);
             cv::namedWindow("right-right matches");
             cv::imshow("right-right matches", xpair_rr_matches_canvas);
             
             
            std::string llxpair_matches_image_name = "llxpairmatches"+std::to_string(frame_count-1)+"-"+std::to_string(0);
            llxpair_image_names.push_back(llxpair_matches_image_name);
            llxpair_images.push_back(xpair_ll_matches_canvas);
            
            std::string lrxpair_matches_image_name = "lrxpairmatches"+std::to_string(frame_count-1)+"-"+std::to_string(0);
            lrxpair_image_names.push_back(lrxpair_matches_image_name);
            lrxpair_images.push_back(xpair_lr_matches_canvas);
         Jess, hope all is well (say hi to Peter btw!). I have a quick question: Some student from the University of Toronto reached out to me on linkedin. I assume he is looking for a job or intersnhip or something of the sort. I just briefly saw his profile and is looking relevant. Could he be of any interest to us? If so, I can copy his linkedin details.
            std::string rlxpair_matches_image_name = "rlxpairmatches"+std::to_string(frame_count-1)+"-"+std::to_string(0);
            rlxpair_image_names.push_back(rlxpair_matches_image_name);
            rlxpair_images.push_back(xpair_rl_matches_canvas);
            
            std::string rrxpair_matches_image_name = "rrxpairmatches"+std::to_string(frame_count-1)+"-"+std::to_string(0);
            rrxpair_image_names.push_back(rrxpair_matches_image_name);
            rrxpair_images.push_back(xpair_rr_matches_canvas);
          */
         }
         
         
         cv::Mat left_features_canvas, right_features_canvas, matches_canvas;
         pstereo_pair->DrawLeftLeftKeypoints(feature_detection_level, left_features_canvas);
         pstereo_pair->DrawRightKeypoints(feature_detection_level, right_features_canvas);
         
         
         pstereo_pair->DrawMatches(feature_detection_level, matches_canvas);
         
         cv::namedWindow("Left features");
         cv::imshow("Left features", left_features_canvas);
         cv::namedWindow("Right features");
         cv::imshow("Right features", right_features_canvas);
         
         cv::namedWindow("Matches");
         cv::imshow("Matches", matches_canvas);
         
         std::string matches_image_name = "lrpairmatches"+std::to_string(frame_count-1);
         lrpair_image_names.push_back(matches_image_name);
         lrpair_images.push_back(matches_canvas);
         
         
         cv::waitKey(-1);
         
         if ( frame_count == n_frames_to_optimize) break;
       
    }
    
    cv::Matx<double, 3, 3> left_K = K1, 
                            //left_R = cv::Matx<double, 3, 3>::eye(),
                            left_Rcv = cv::Matx<double, 3, 3>( 9.9999e-01 , 5.3940e-03 , 9.2253e-05, -5.3944e-03 , 9.9997e-01 , 4.9194e-03, -6.5715e-05, -4.9198e-03, 9.9999e-01 ),
                            left_R1 = cv::Matx<double, 3, 3>( 0.935169, 0.253237, -0.247649, -0.265801, 0.963858, -0.018107, 0.234113, 0.082758, 0.968681),
                            right_R1 = cv::Matx<double, 3, 3>(0.939511, 0.234068, -0.250061, -0.249245, 0.967963, -0.030391, 0.234937, 0.090879, 0.967753),
                            right_Rcv = cv::Matx<double, 3, 3>(9.9988e-01, -1.5360e-02, -4.2212e-04, 1.5358e-02 , 9.9987e-01, -4.9230e-03, 4.9768e-04 , 4.9160e-03 , 9.9999e-01 ), 
                            left_R = cv::Matx<double, 3, 3>( 0.9810653386024667, -0.05303547448340697, -0.18627409868027, 0.05851849271093656, 0.9979964089871302, 0.02405729950831258, 0.184624991276011, -0.03450226217384129, 0.982203240933983),
                            right_R = cv::Matx<double, 3,3>(0.9805925878907555, -0.0801337765553905, -0.1789322621236364, 0.08557655343329917, 0.9960684626747915, 0.02289696851452299, 0.1763939626976747, -0.0377650039010737, 0.9835949239418447),
                           right_K = K2; 
                           //right_R = cv::Matx<double, 3, 3>(0.9997160, -0.0236696, -0.0027860, 0.0236445, 0.9996822, -0.0087446, 0.0029921, 0.0086762, 0.9999579);
                           //right_R = R2;
    cv::Vec<double, 5> left_dist = dist1, right_dist = dist2;
    
    /*
    // Test again the %$##^&%#$^&# projecton matrices
    online_calibration::PinholeCamera<5, double> left_camera( cv::Size_<int>(2048, 1080), left_K, left_Rcv, cv::Vec<double, 3>(0, 0, 0), left_dist );
    online_calibration::PinholeCamera<5, double> right_camera( cv::Size_<int>(2048, 1080), right_K, right_Rcv, cv::Vec<double, 3>(214.631483401779/1855.2965066509, 0, 0), right_dist );
    
    cv::Mat P1, P2, Rect1(left_camera.RotationMatrix()), Rect2(right_camera.RotationMatrix());
    online_calibration::StereoFrameWrapper::OpenCVProjectionMatrices(left_camera, right_camera, Rect1, Rect2, P1, P2);
    cv::Matx<double, 3, 4> leftP, rightP;
    left_camera.SetRotationMatrix(left_camera.RotationMatrix());
    right_camera.SetRotationMatrix(right_camera.RotationMatrix());
    online_calibration::ComputeOpenCVProjectionMatrix(left_camera, right_camera, leftP, rightP);
    
    std::cout << " Rectification R1 : " << Rect1 << std::endl;
    std::cout << " Rectification R2 : " << Rect2 << std::endl;
    std::cout << " ^&**$* Projection P1 : " << P1 << std::endl;
    std::cout << " ^&*%^& Projection P2 : " << P2 << std::endl;
    
    std::cout << " Projection P1 WITHOUT the OpenCV rectification: " << leftP << std::endl;
    std::cout << " Projection P2 WITHOUT the OpenCV rectification: " << rightP << std::endl;
    
    // Finding optimal absolute rotations
    
    absolute_orientation::RotationAveraging<double> rotavg( std::vector<cv::Matx<double, 3, 3>>({left_R1, right_R1}) );
    
    rotavg.Initialize();
    rotavg.Solve();
    
    std::cout << " The averaged transformation : " << rotavg.R() << std::endl;
    std::cout << " The matrix S_ : " << rotavg.SVD_S() << std::endl;
    std::cout << " The matrix U_ : " << rotavg.SVD_U() << std::endl;
    std::cout << " The matrix Vt_ : " << rotavg.SVD_Vt() << std::endl;
    std::cout << " The dsata matrix A_ : " << rotavg.A() << std::endl;
    
    exit(0);
    */
    // Create a PointCorrespondences object for the stereo pairs
    // Generate matches across all images 
    correspondences.GenerateMatches();
    // Register the correspondences for quick access
    correspondences.RegisterCorrespondences();   
    // Draw the correpsonces in pairs of stereo pairs
    std::vector<cv::Mat> corresponding_pair_images;
    std::vector<std::string> corresponding_pair_names;
    for (size_t k = 0; k < correspondences.StereoPairsSize(); k++)
    {
        for (size_t n = 0; n < k; n++)
        {
            cv::Mat mmatch_canvas;
            std::string corresponding_pair_name = "correspondences_pair" + std::to_string(n)+"_"+std::to_string(k);
            
            correspondences.DrawCorrespondences(n, k , mmatch_canvas);
            
            corresponding_pair_names.push_back(corresponding_pair_name);
            corresponding_pair_images.push_back(mmatch_canvas);
            
            cv::namedWindow("cor");
            cv::imshow("cor", mmatch_canvas);
            cv::waitKey(-1);
        }
    }
    
    // Save the match images
    SaveImages(calibration_data_path + "//output", lrpair_image_names, lrpair_images);
    SaveImages(calibration_data_path + "//output", lrxpair_image_names, lrxpair_images);
    SaveImages(calibration_data_path + "//output", llxpair_image_names, llxpair_images);
    SaveImages(calibration_data_path + "//output", rlxpair_image_names, rlxpair_images);
    SaveImages(calibration_data_path + "//output", llxpair_image_names, llxpair_images);
    SaveImages(calibration_data_path + "//output", rrxpair_image_names, rrxpair_images);
    SaveImages(calibration_data_path + "//output", corresponding_pair_names, corresponding_pair_images);

    
    
    online_calibration::BundleOptimizer optimizer(correspondences);
    optimizer.Initialize();
    //optimizer.RunRigPosesOptimization();
    optimizer.RunRigPosesBundleOptimization();
    
    //std::cout <<" Printing out left rotation : " << optimizer.LeftOptimizedRotation<double>() << std::endl;
    //std::cout <<" Printing out right rotation : " << optimizer.RightOptimizedRotation<double>() << std::endl;
    //std::cout <<" Printing out baseline : " << optimizer.OptimizedBaseline<double>() << std::endl;
    //std::cout <<" Printing out initial LEFT intrinsics : " << optimizer.LeftInitialIntrinsics<double>() << std::endl;
    //std::cout <<" Printing out initial RIGHT intrinsics : " << optimizer.RightInitialIntrinsics<double>() << std::endl;
    //std::cout <<" Printing out optimized LEFT intrinsics : " << optimizer.LeftOptimizedIntrinsics<double>() << std::endl;
    //std::cout <<" Printing out optimized RIGHT intrinsics : " << optimizer.RightOptimizedIntrinsics<double>() << std::endl;
    
    
    /*
    // Run the filter now 3 times (for each pair of images)
    
    cv::Matx<float, 3, 3>& K1_start = K1_air;
    cv::Matx<float, 3, 3>& K2_start = K2_air;
    cv::Matx<float, 3, 3>& R1_start = R1_air;
    cv::Matx<float, 3, 3>& R2_start = R2_air;
    cv::Vec<float, 3>& t1_start = t1_air;
    cv::Vec<float, 3>& t2_start = t2_air;
    
    cv::Vec<float, 5>& dist1_start = dist1_air;
    cv::Vec<float, 5>& dist2_start = dist2_air;
    
    
    // projection image pairs
    std::vector<cv::Mat> right_projection_imgs;
    std::vector<cv::Mat> left_projection_imgs;
    // projection image names
    std::vector<std::string> right_image_names;
    std::vector<std::string> left_image_names;
    // List of best particles
    std::vector<online_calibration::OptimizerParticle> best_particles;
    // names of files to save the parameters (camera parameters, relative pose (stereo rig), odometry, depths )
    std::vector<std::string> best_particle_params_names;
    
    // at which pyramid level are we displaying projections?
    size_t projection_img_level = 0;
    
    for (size_t k = 0; k < n_frames_to_optimize; k++)
    {
        if (k > 0 ) 
        {
            pf.AdvancePose();
        }
        else 
        {
            pf.Initialize(K1_start, R1_start, t1_air, dist1_start, K2_start, R2_start, t2_start, dist2_start);
        }

        pf.UpdateWeightsAndResample();
        
        
        //pf.DrawPointCloud(pf.BestParticle());
        
        
        
        // display matches in stereo pairs from base pair amd up to the current one
        cv::Mat right_projection_img, left_projection_img;
    
        for (size_t i = 0; i <= k; i++)
        {
            std::string right_window_name = "right projections_step"+std::to_string(k)+"_pair"+std::to_string(i);
            std::string left_window_name = "left projections_step"+std::to_string(k)+"_pair"+std::to_string(i);
            
            std::string right_image_name = "right_step"+std::to_string(k)+"_pair"+std::to_string(i);
            std::string left_image_name = "left_step"+std::to_string(k)+"_pair"+std::to_string(i);
        
            cv::namedWindow(right_window_name);
            cv::namedWindow(left_window_name);
    
            pf.DrawRightProjections(i, projection_img_level, pf.BestParticle(), right_projection_img);
                
            pf.DrawLeftProjections(i, projection_img_level, pf.BestParticle(), left_projection_img);
            
            cv::imshow(right_window_name, right_projection_img);
            cv::imshow(left_window_name, left_projection_img);
            
            right_projection_imgs.push_back(right_projection_img);
            left_projection_imgs.push_back(left_projection_img);
            right_image_names.push_back(right_image_name);
            left_image_names.push_back(left_image_name);
            
        }
            
        std::string best_particle_params_name = "best_params_step"+std::to_string(k);
        best_particles.push_back(pf.BestParticle());
        best_particle_params_names.push_back(best_particle_params_name);
        
        cv::waitKey(500);
                
    }
            
                
    auto inference = pf.Inference();
    std::cout <<"\n\tPF inference mean: " << inference.first << std::endl << " and PF inference max. particle: " << inference.second << std::endl;
            
    std::cout << std::endl;
    std::cout << " Initial Intrinsics #1 : " << K1_start << std::endl;
    std::cout << " Initial Intrinsics #2 : " << K2_start << std::endl;
            
    pf.DrawCloudMesh(pf.BestParticle());
            
        
        
    cv::waitKey(-1);
        
    SaveImages(home_dir + "//calibration_data//output", left_image_names, left_projection_imgs);
    SaveImages(home_dir + "//calibration_data//output", right_image_names, right_projection_imgs);

    
    
    SaveParams(home_dir + "//calibration_data//output", best_particle_params_names, best_particles);
    */
    return 1;
}
