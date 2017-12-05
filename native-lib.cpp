#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <android/bitmap.h>
#include <opencv2/core/types.hpp>
#include <android/native_window.h>
#include <android/native_window_jni.h>

/*
 *The input image from camera and reference image is used as input to estimate the Pose and
 * draw a line on the input image. The resulting image is returned as output array of pixels
 */

// the File has only one function hence, no function level comments

extern "C" {
JNIEXPORT jintArray JNICALL
Java_com_example_neera_once_1more_ASUForia_nativePoseEstimation(
        JNIEnv *env,
        jobject This,
        jintArray pixels_,
        jintArray pixels_ref_,
        jint width, jint height,jint w_l,jint h_1) {
    //get input image as an array of int
    jint *pixels = env->GetIntArrayElements(pixels_, NULL);
    long length = env->GetArrayLength(pixels_);

    // get reference image as an of int
    jint *pixels_ref = env->GetIntArrayElements(pixels_ref_,NULL);
    //set output array as input array ---- the no. of pixels of input=no.of pixels of output
    jintArray pixelsOut = env->NewIntArray(length);
    //set intermediate array also similarly
    jint *pixels_check =  env->GetIntArrayElements(pixelsOut, NULL);


//--------convert pixel array into cv::MAT format-----------------------
    cv::Mat in = cv::Mat(height+height/2,width,CV_8UC1,pixels);
    cv::Mat in_ref = cv::Mat(h_1+h_1/2,w_l,CV_8UC1,pixels_ref);
//-------- END convert pixel array into cv::MAT format-----------------------

//--------Detect Features and extract descriptors of the reference image-----------------------
    std::vector<cv::KeyPoint> ref_key ;
//compute Keypoints using ORB Feature Detector
    cv::Ptr<cv::FeatureDetector >detector_ref = cv::ORB::create(2000) ;
    if( in_ref.rows ) {
        detector_ref->detect(in_ref, ref_key);
    }

     cv::Mat descriptors_frame_ref;
    cv::Ptr<cv::DescriptorExtractor>  extractor_ref;
     extractor_ref = cv::ORB::create(2000);
    extractor_ref->compute(in_ref,ref_key,descriptors_frame_ref);
//--------END Detect Features and extract descriptors of the reference image-----------------------

//--------Detect Features and extract descriptors of the input image-----------------------
    std::vector<cv::KeyPoint> key ;
//compute Keypoints using ORB Feature Detector
    cv::Ptr<cv::FeatureDetector >detector = cv::ORB::create(2000) ;
   if( in.rows ) {
        detector->detect(in, key);
    }

    cv::Mat descriptors_frame;
    cv::Ptr<cv::DescriptorExtractor>  extractor;
    extractor = cv::ORB::create(2000);
    extractor->compute(in,ref_key,descriptors_frame);
//-------- END Detect Features and extract descriptors of the input image-----------------------

    cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12,1); // instantiate LSH index parameters
    cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search parameters
    // instantiate FlannBased matcher
    cv::DescriptorMatcher * matcher = new cv::FlannBasedMatcher(indexParams, searchParams);
    std::vector<cv::DMatch> matches;
    //Match descriptors of ref. image and input image
    matcher->match(descriptors_frame_ref,descriptors_frame,matches);

   // std::vector<std::vector<cv::DMatch>> matches_1,matches_2;
   // matcher->knnMatch(descriptors_frame_ref,descriptors_frame,matches_1,2);
   // matcher->knnMatch(descriptors_frame,descriptors_frame_ref,matches_2,2);
    double max_dist = 0; double min_dist = 100;

//-----------------Finding only the good matches from all the matches----------------------------------------
    for( int i = 0; i < descriptors_frame_ref.rows; i++ )
    {//Finding the minimum and maximum distance between matches
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
        //instantiating good_matches
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < descriptors_frame_ref.rows; i++ )
    {
        //if distance is less than 2*min_distance or less than a fixed value then
        // it is a good match
        if(2*min_dist>0.02)
        {
            if( matches[i].distance <= 2*min_dist )
            { good_matches.push_back( matches[i]); }
        }
        else
        {
            if( matches[i].distance <= 0.02 )
            { good_matches.push_back( matches[i]); }
        }


    }
//----------------- END Finding only the good matches from all the matches----------------------------------------

    //initializing the camera parameters
    double focal_length = in.cols; // Approximate focal length.
    double x1 =(in.cols/2);
    double y1 =(in.rows/2);
    //intiailizing the camera matrix
    cv::Mat camera_matrix = cv::Mat::zeros(3,3,CV_64FC1);
    camera_matrix.at<double>(0, 0) = focal_length;       //      [ fx   0  cx ]
    camera_matrix.at<double>(1, 1) = focal_length;       //      [  0  fy  cy ]
    camera_matrix.at<double>(0, 2) = x1;                 //      [  0   0   1 ]
    camera_matrix.at<double>(1, 2) = y1;
    camera_matrix.at<double>(2, 2) = 1;

    //intializing the distortion coefficients assuming no lens distortion
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type);

    // container for the model 3D coordinates found in the scene
    std::vector<cv::Point3f> list_points3d_model_match;
    // container for the model 2D coordinates found in the scene
    std::vector<cv::Point2f> list_points2d_scene_match;
    for(unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
    {
        //cv::Point3f point3d_model = list_points3d_model[ matches[match_index].trainIdx ];   // 3D point from model
        cv::Point2f point2d_scene = key[ good_matches[match_index].queryIdx ].pt;    // 2D point from the scene
        cv::Point2f point2d_train = ref_key[good_matches[match_index].trainIdx].pt; // 3D point from ref image part 1
        //list_points3d_model_match.push_back(cv::Point3d(20.0f, 25.0f, 0.0f));
        //list_points3d_model_match.push_back(cv::Point3d(point2d_scene.x,point2d_scene.y, 0.0f));        // add 3D point
        list_points3d_model_match.push_back(cv::Point3d(point2d_train.x,point2d_train.y, 0.0f)); // 3D point from ref image part 2
        list_points2d_scene_match.push_back(point2d_scene);                                      // add 2D point
    }
//--------------- Calculating rvec and tvec-------------------------------------------------------------
    // RANSAC parameters
    int iterationsCount = 500;        // number of Ransac iterations.
    float reprojectionError = 2.0;    // maximum allowed distance to consider it an inlier.
    float confidence = 0.95;          // RANSAC successful confidence.
    //initializing rvec and tvec
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector

    bool useExtrinsicGuess = false;
    //using solvePnPRansac to calculate rvec and tvec
    cv::solvePnPRansac(list_points3d_model_match,list_points2d_scene_match,camera_matrix,dist_coeffs,rvec,tvec,useExtrinsicGuess,iterationsCount,reprojectionError,confidence);

//--------------- END Calculating rvec and tvec-------------------------------------------------------------

//---------------------Drawing line on input image -----------------------------------------------------
   std::vector<cv::Point3d> line_end_point3D;
    std::vector<cv::Point2d> line_end_point2D;
    //fixing points to draw line
    line_end_point3D.push_back(cv::Point3d(0,0,500));
    line_end_point3D.push_back(cv::Point3d(0,0,0));
    // nose_end_point3D.push_back(cv::Point3d(0,500,100.0));
    //nose_end_point3D.push_back(cv::Point3d(100,500,0));
    //nose_end_point3D.push_back(cv::Point3d(100,500,100.0));
    //float i = (float) *rvec.data;
    //float j = (float) *tvec.data;

    //projecting points in 2D to draw line
    projectPoints(line_end_point3D, rvec, tvec, camera_matrix, dist_coeffs, line_end_point2D);

       // cv::circle(in,nose_end_point2D.pt,1000,cv::Scalar(0,255,0),10,8);
    //drawing line on input image
  cv::line(in,line_end_point2D[1], line_end_point2D[0], cv::Scalar(0,255,0), CV_AA);
    //cv::line(in,list_points2d_scene_match[1], nose_end_point2D[3], cv::Scalar(0,255,0), CV_AA);
   // cv::line(in,list_points2d_scene_match[3], nose_end_point2D[2], cv::Scalar(0,255,0), CV_AA);
    //cv::line(in,list_points2d_scene_match[2], nose_end_point2D[0], cv::Scalar(0,255,0), CV_AA);

    //cv::Mat out;
  //  cv::drawMatches(in_ref,ref_key,in,key,matches,out);

    //for (int i =0;i<key.size();i++)
    //{
     //   cv::KeyPoint lol = key[i];
     //   cv::circle(in,cv::Point(lol.pt.x,lol.pt.y),10,cv::Scalar(255,0,255));
   // }
//---------------------END Drawing line on input image -----------------------------------------------------
    //storing data into intermediate array
   int size = in.total() * in.elemSize();
    std::memcpy(pixels_check,in.data,size * sizeof(pixels_check));

    //pixels_check=pixels;
//returning jintArray to MainActivity
    env->SetIntArrayRegion(pixelsOut, 0, length, pixels_check);
    env->ReleaseIntArrayElements(pixels_, pixels, 0);
    return pixelsOut;
    //pixelsOut[length] = pixels[length];

}
}
