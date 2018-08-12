#include <iostream>

#include "boost/program_options.hpp"
#include "boost/filesystem.hpp"

#include "FastPhOdo.h"

int ListSequenceDirectory( std::string & directory, std::vector< std::string > & sequenceFrames )
{
  boost::filesystem::path sequencePath( directory );
  if( !boost::filesystem::is_directory( sequencePath ) )
  {
    return EXIT_FAILURE;
  }
  
  sequenceFrames.clear();
  for( auto & file: boost::filesystem::directory_iterator( sequencePath ) )
  {
    if( file.path().extension().string() == ".png" ||
        file.path().extension().string() == ".jpg" ||
        file.path().extension().string() == ".bmp" )
    {
      sequenceFrames.push_back( file.path().string() );
    }
  }
  return EXIT_SUCCESS;
}

void GenerateOdometryTrajectoryImage( cv::Mat & trajectoryImage, cv::Mat & translationVector )
{
  // Plot trajectory in X-Z plane
  int x = int( translationVector.at< double >( 0 ) ) + ( trajectoryImage.cols / 2 );
  int y = -int( translationVector.at< double >( 2 ) ) + ( trajectoryImage.rows / 2 );
  cv::circle( trajectoryImage, cv::Point( x, y ), 1, CV_RGB( 0, 0, 255 ), 1 );
}

int parseInputArguments( int argc, char* argv[],
                         std::string & sequenceDirectory,
                         double & focalLength,
                         double & principalPointX,
                         double & principalPointY )
{
  int res = EXIT_SUCCESS;

  boost::program_options::options_description description("Options");

  description.add_options()
    ( "help", "Produce help message" )
    ( "sequence-directory", boost::program_options::value< std::string >( & sequenceDirectory ), "Input sequence directory" )
    ( "focal-length", boost::program_options::value< double >( & focalLength ), "Calibrated camera focal length" )
    ( "principal-point-x", boost::program_options::value< double >( & principalPointX ), "Calibrated camera principal point X" )
    ( "principal-point-y", boost::program_options::value< double >( & principalPointY ), "Calibrated camera principal point Y" )
  ;

  boost::program_options::variables_map variableMap;
  boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description ), variableMap );
  boost::program_options::notify( variableMap );

  if( variableMap.count("help") )
  {
    std::cout << description << std::endl;
    return EXIT_FAILURE;
  }

  if( !variableMap.count("sequence-directory") )
  {
    std::cerr << "Input sequence directory not provided" << std::endl;
    return EXIT_FAILURE;
  }

  if( !variableMap.count("focal-length") || !variableMap.count("principal-point-x") || !variableMap.count("principal-point-y") )
  {
    std::cerr << "Input camera parameters not provided" << std::endl;
    return EXIT_FAILURE;
  }

  return res;
}

int main( int argc, char** argv )
{
  typedef FastPhOdo                FastPhOdoType;
  typedef FastPhOdoType::ImageType ImageType;

  // Parse the input arguments
  std::string sequenceDirectory;
  double focalLength;
  double principalPointX;
  double principalPointY;
  if( parseInputArguments( argc, argv,
                           sequenceDirectory,
                           focalLength,
                           principalPointX,
                           principalPointY ) )
  {
    return EXIT_FAILURE;
  }

  // Create photometric odometry estimator with input camera calibration values
  FastPhOdoType photometricOdometry;
  photometricOdometry.SetCameraFocalLength( focalLength );
  photometricOdometry.SetCameraPrincipalPoint( cv::Point2d( principalPointX, principalPointY ) );

  // List sequence directory
  std::vector< std::string > sequenceFrames;
  if( ListSequenceDirectory( sequenceDirectory, sequenceFrames ) )
  {
    return EXIT_FAILURE;
  }

  // Initialize odometry estimator
  ImageType bootstrappingFrame = cv::imread( sequenceFrames[0], 0 );
  ImageType initialFrame = cv::imread( sequenceFrames[1], 0 );
  photometricOdometry.SetInitialFrames( bootstrappingFrame, initialFrame );

  cv::Mat cumulativeRotation = cv::Mat::eye( 3, 3, CV_64F );
  cv::Mat cumulativeTranslation = cv::Mat::zeros( 3, 1, CV_64F );

  // Prepare verbose stuff
  cv::namedWindow( "Odometry trajectory", cv::WINDOW_AUTOSIZE );
  cv::Mat odometryTrajectory = cv::Mat( 480, 640, CV_8UC3 );
  odometryTrajectory = cv::Scalar( 255, 255, 255 );
  cv::namedWindow( "Frame", cv::WINDOW_AUTOSIZE );

  // Iterate over available sequence frames
  for( size_t frameIt = 2; frameIt < sequenceFrames.size(); frameIt++ )
  {
    // Estimate odometry
    photometricOdometry.UpdateOdometry();
    cumulativeTranslation = cumulativeTranslation + cumulativeRotation * photometricOdometry.GetTranslationVector();
    cumulativeRotation = photometricOdometry.GetRotationMatrix() * cumulativeRotation;

    //std::cout << "Rotation: " << std::endl;
    //std::cout << cumulativeRotation << std::endl;

    //std::cout << "Translation: " << std::endl;
    //std::cout << cumulativeTranslation << std::endl;

    // View results
    GenerateOdometryTrajectoryImage( odometryTrajectory, cumulativeTranslation );
    cv::imshow( "Odometry trajectory", odometryTrajectory );
    cv::imshow( "Frame", photometricOdometry.GetCurrentFrame() );
    cv::waitKey( 1 );

    // Update frame for next estimation
    ImageType currentFrame = cv::imread( sequenceFrames[frameIt], 0 );
    photometricOdometry.UpdateFrame( currentFrame );
  }

  return EXIT_SUCCESS;
}