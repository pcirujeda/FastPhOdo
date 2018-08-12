#ifndef __FastPhOdo_hxx
#define __FastPhOdo_hxx

#include "FastPhOdo.h"

FastPhOdo
::FastPhOdo()
{}

FastPhOdo
::~FastPhOdo()
{}

void
FastPhOdo
::SetInitialFrames( const ImageType & bootstrappingFrame, const ImageType & initialFrame )
{
  this->m_PreviousFrame = bootstrappingFrame.clone();
  this->m_CurrentFrame = initialFrame.clone();
}

void
FastPhOdo
::SetCameraFocalLength( const double focalLength )
{
  this->m_CameraFocalLength = focalLength;
}

void
FastPhOdo
::SetCameraPrincipalPoint( const cv::Point2d & principalPoint )
{
  this->m_CameraPrincipalPoint = principalPoint;
}

void
FastPhOdo
::UpdateFrame( const ImageType & newFrame )
{
  this->m_PreviousFrame = this->m_CurrentFrame.clone();
  this->m_CurrentFrame = newFrame.clone();
}

cv::Mat
FastPhOdo
::GetCurrentFrame()
{
  return this->m_CurrentFrame;
}

cv::Mat
FastPhOdo
::GetRotationMatrix()
{
  return this->m_CurrentRotation;
}

cv::Mat
FastPhOdo
::GetTranslationVector()
{
  return this->m_CurrentTranslation;
}

void
FastPhOdo
::UpdateOdometry()
{
  // Detect keypoint locations and track their frame-to-frame trajectory
  this->DetectFeaturePoints( this->m_PreviousFrame, this->m_PreviousFrameFeatureCoordinates );
  this->DetectFeaturePoints( this->m_CurrentFrame, this->m_CurrentFrameFeatureCoordinates );
  this->TrackFeaturePoints();

  // Find essential matrix
  cv::Mat dummyMask;
  this->m_EssentialMatrix = cv::findEssentialMat( this->m_CurrentFrameFeatureCoordinates, this->m_PreviousFrameFeatureCoordinates,
                                                  this->m_CameraFocalLength, this->m_CameraPrincipalPoint,
                                                  cv::RANSAC, 0.999, 1.0, dummyMask );
  
  // Estimate pose from essential matrix and keypoint coordinates
  cv::recoverPose( this->m_EssentialMatrix,
                   this->m_CurrentFrameFeatureCoordinates, this->m_PreviousFrameFeatureCoordinates,
                   this->m_CurrentRotation, this->m_CurrentTranslation,
                   this->m_CameraFocalLength, this->m_CameraPrincipalPoint, dummyMask );
}

void
FastPhOdo
::DetectFeaturePoints( const ImageType & frame, PointContainerType & featureCoordinates )
{
  // Detect FAST keypoints and get their image plane coordinates
  std::vector< cv::KeyPoint > frameKeypoints;
  cv::FAST( frame, frameKeypoints, 20, true );
  cv::KeyPoint::convert( frameKeypoints, featureCoordinates, std::vector< int >() );
}

void
FastPhOdo
::TrackFeaturePoints()
{
  // Compute feature coordinates consistent temporal tracking
  std::vector< uchar > featureCoordinateStatusContainer;
  std::vector< float > errorContainer;          
  cv::TermCriteria terminationCriteria = cv::TermCriteria( cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01 );

  // Get optical flow field for detected keypoints
  cv::calcOpticalFlowPyrLK( this->m_PreviousFrame, this->m_CurrentFrame,
                            this->m_PreviousFrameFeatureCoordinates, this->m_CurrentFrameFeatureCoordinates,
                            featureCoordinateStatusContainer, errorContainer, cv::Size( 21, 21 ), 3, terminationCriteria, 0, 0.001 );


  PointContainerType trackedPreviousFrameFeatureCoordinates;
  PointContainerType trackedCurrentFrameFeatureCoordinates;
  for( size_t fcsIt = 0; fcsIt < featureCoordinateStatusContainer.size(); fcsIt++ )
  {
    cv::Point2f currentPoint = this->m_CurrentFrameFeatureCoordinates.at( fcsIt );

    // Check current keypoint is within frame boundaries
    if( ( currentPoint.x < 0 ) || ( currentPoint.y < 0 ) ||
        ( currentPoint.x > this->m_CurrentFrame.rows ) || ( currentPoint.y > this->m_CurrentFrame.cols ) )
    {
      featureCoordinateStatusContainer.at( fcsIt ) = 0;
    }

    // If current keypoint is valid and within image bounds, keep it
    if( featureCoordinateStatusContainer.at( fcsIt ) != 0 )
    {
      trackedPreviousFrameFeatureCoordinates.push_back( this->m_PreviousFrameFeatureCoordinates[ fcsIt ] );
      trackedCurrentFrameFeatureCoordinates.push_back( this->m_CurrentFrameFeatureCoordinates[ fcsIt ] );
    }
  }

  // Swap keypoint coordinate containers with filtered ones
  this->m_PreviousFrameFeatureCoordinates.clear();
  this->m_CurrentFrameFeatureCoordinates.clear();

  this->m_PreviousFrameFeatureCoordinates.swap( trackedPreviousFrameFeatureCoordinates );
  this->m_CurrentFrameFeatureCoordinates.swap( trackedCurrentFrameFeatureCoordinates );
}

#endif
