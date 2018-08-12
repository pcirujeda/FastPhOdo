#ifndef __FastPhOdo_h
#define __FastPhOdo_h

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

class FastPhOdo
{
  public:
    typedef cv::Mat ImageType;

    FastPhOdo();
    ~FastPhOdo();

    void SetInitialFrames( const ImageType & bootstrappingFrame, const ImageType & initialFrame );
    void SetCameraFocalLength( const double focalLength );
    void SetCameraPrincipalPoint( const cv::Point2d & principalPoint );

    void UpdateFrame( const ImageType & newFrame );
    void UpdateOdometry();

    ImageType GetCurrentFrame();
    cv::Mat GetRotationMatrix();
    cv::Mat GetTranslationVector();

  private:
    typedef std::vector< cv::Point2f > PointContainerType;

    ImageType          m_PreviousFrame;
    ImageType          m_CurrentFrame;
    PointContainerType m_PreviousFrameFeatureCoordinates;
    PointContainerType m_CurrentFrameFeatureCoordinates;
    double             m_CameraFocalLength;
    cv::Point2d        m_CameraPrincipalPoint;
    cv::Mat            m_EssentialMatrix;
    cv::Mat            m_CurrentRotation;
    cv::Mat            m_CurrentTranslation;

    // Internal methods
    void DetectFeaturePoints( const ImageType & frame, PointContainerType & featureCoordinates );
    void TrackFeaturePoints();
};

#include "FastPhOdo.hxx"
#endif
