// 
// George Terzakis, 17 July 2022
//
// Mono-Calibration from images
//

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <random>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

/**
 * @brief Get filenames in directory
 * 
 **/
bool IsDirectory(const std::string& path) 
{
   struct stat statbuf;
   if (stat(path.c_str(), &statbuf) != 0)
       return 0;
   return S_ISDIR(statbuf.st_mode) != 0;
}

std::vector<std::string> GetFilenamesInDir(const std::string& dir_name, const bool include_dirs = false)
{
    std::vector<std::string> filenames;
    
    if (dir_name.length() == 0) { return filenames; }
    
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir (dir_name.c_str())) != NULL) 
    {
        while ((ent = readdir(dir)) != NULL) 
        {
            //printf ("%s\n", ent->d_name);
            std::string full_filename = dir_name + (dir_name.back() == '/' ? "" : "/")+std::string(ent->d_name); 
            if (!include_dirs && IsDirectory(full_filename)) { continue; }
            filenames.push_back(full_filename);
            std::cout << full_filename << std::endl;
            
        }
        closedir (dir);
    } else 
    {
    // could not open directory */
    //perror ("");
    //return false;
    }
    return filenames;
}


/**
 * Save monocular calibration a la ROS:
 *
 *   image_width: wwwww
 *   image_height: hhhhh
 *   camera_name: nnnnnnn
 *   camera_matrix (3x3): 
 *   distortion_model: plumb_bob
 *   distortion_coefficients (1x5):
 *   rectification_matrix:
 *   projection_matrix:
 *
 */
void SaveMonoCalibration(
    const std::string& filename, 
    const cv::Size& image_size,
    const cv::Mat& camera_matrix, 
    const cv::Mat& distortion_coefficients,
    const std::string& distortion_model = "plumb_bob",
    const std::string& camera_name = "monocam" )
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
    fs << "image_with" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "camera_name" << camera_name;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_model" << distortion_model;
    fs << "distortion_coefficients" << distortion_coefficients;
    
    // Rectification is the identity and projection is same as camera matrix 
    cv::Mat rectification_matrix = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat projection_matrix(3, 4, CV_32F);
    cv::Mat _camera_matrix = cv::Mat_<float>(camera_matrix);
    for (size_t r = 0; r < 3; r++)
    {
        projection_matrix.at<float>(r, 3) = 0;
        for (size_t c = 0; c < 3; c++) 
        {
            projection_matrix.at<float>(r, c) = _camera_matrix.at<float>(r, c);
        }
    }
    
    fs << "rectification_matrix" << rectification_matrix;
    fs << "projection_matrix" << projection_matrix;
    
    fs.release();
}


/**
 * Load monocular calibration a la ROS:
 *
 *   image_width: wwwww
 *   image_height: hhhhh
 *   camera_name: nnnnnnn
 *   camera_matrix (3x3): 
 *   distortion_model: plumb_bob
 *   distortion_coefficients (1x5):
 *   rectification_matrix:
 *   projection_matrix:
 *
 */
void LoadMonoCalibration(
    const std::string& filename, 
    cv::Size& image_size,
    cv::Mat& camera_matrix, 
    cv::Mat& distortion_coefficients,
    cv::Mat& rectification_matrix,
    cv::Mat& projection_matrix,
    std::string& distortion_model,
    std::string& camera_name )
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
        
    fs << "image_with" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "camera_name" << camera_name;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_model" << distortion_model;
    fs << "distortion_coefficients" << distortion_coefficients;
    
}

double PointDistance(size_t index, 
                     const std::vector<cv::Point2f>& points1, 
                     const std::vector<cv::Point2f>& points2)
{
    assert(index < points1.size() && points1.size() == points2.size() );
    
    double dist = sqrt( ( points1[index].x - points2[index].x ) * ( points1[index].x - points2[index].x ) +
                        ( points1[index].y - points2[index].y ) * ( points1[index].y - points2[index].y ) );
    return dist;
}

int main()
{
    
    
    
    // 
    int max_images = 90;
    std::vector<std::string> image_filenames = GetFilenamesInDir("/home/george/sandbox/voxelvision_matlab/calib2_L");
    
    std::vector<std::vector<cv::Point2f>> all_corners;
    
    const cv::Size_<int> board_size = cv::Size_<int>(8, 6);
    
    // Create a vector with the chessboard points in a local frame
    std::vector<std::vector<cv::Point3f>> object_points;
    object_points.push_back(std::vector<cv::Point3f>());
    float square_dim = 0.032;
    for (int r = 0; r < board_size.height; r++)
    {
        for (int c = 0; c < board_size.width; c++)
        {
            float x = c * square_dim, y = r * square_dim;
            object_points[0].push_back(cv::Point3f(x, y, 0));
        }
    
    }
    
    
    
    cv::Size_<int> image_size;
    int num_registered_images = 0;
    int skip_counter = 0;
    int counter = 0;
    std::vector<cv::Point2f> previous_corners;
    
    while (num_registered_images < max_images && counter < image_filenames.size())
    {
        
        cv::Mat image = cv::imread(image_filenames[counter], cv::IMREAD_GRAYSCALE );
        counter++;
        
        image_size = image.size();
        cv::Mat_<uchar> gray_image(image.rows, image.cols);
        image.copyTo(gray_image);
        
        std::vector<cv::Point2f> corners;
        bool patternfound = cv::findChessboardCorners(
            gray_image, 
            board_size, 
            corners, 
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK );
        
        if(patternfound)
        {
            
            
            cv::cornerSubPix(
                gray_image, 
                corners, 
                cv::Size(11, 11), 
                cv::Size(-1, -1), 
                cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1) );
            
            all_corners.push_back(corners);
            num_registered_images++;
            cv::drawChessboardCorners(gray_image, board_size, corners, patternfound);
            
        }
        else
        {
            std::cerr << "\n Could not find any corners!!!\n";
        }
        
        cv::namedWindow("capture");
        cv::imshow("capture", gray_image);
        cv::waitKey(5);
    }
    
    
    
    //
    // Filling-in the remaining object points
    for (int i = 1; i < all_corners.size(); i++)
    {
        object_points.push_back(object_points[0]);
    }
    
    //
    std::cout << "\n\t Number of points on the chessbard : " << board_size.width << " x " << board_size.height << " x " << all_corners.size() << " = " << object_points[0].size() << " x " << all_corners.size() << std::endl;
    std::cout << "\n\tNumber of chessboards captured : " << all_corners.size()<<std::endl;
    std::cout << "\n\t Calibrating ...\n";
    
    
    std::vector<cv::Mat> Rvecs, tvecs;
    cv::Mat mK, mdist;
    cv::calibrateCamera(object_points, all_corners, image_size, mK, mdist, Rvecs, tvecs );
    
    std::cout << " Image size (rows, cols) : " << image_size.height << " , " <<image_size.width <<" ).\n";
    std::cout << "\n\tCalibrated camera intrinsics : " << mK << std::endl;
    std::cout << "\n\tCalibrated camera distortion coefficients : " << mdist << std::endl;
    
    SaveMonoCalibration("tatiana_calib", image_size, mK, mdist);
}
