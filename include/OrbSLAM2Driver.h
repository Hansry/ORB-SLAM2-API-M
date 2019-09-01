#ifndef ORBSLAMDRIVER_H
#define ORBSLAMDRIVER_H

#include <string>
#include "thread"
#include <opencv2/core/core.hpp>
#include "System.h"
#include <Eigen/Core>

namespace drivers{

///@brief Mat->Eigen
Eigen::Matrix4f MatToEigen(const cv::Mat& mat_matrix);

///@brief Eigen->Mat
cv::Mat EigenToMat(const Eigen::Matrix4f &eigen_matrix);

// \brief DynSLAM和ORBSLAM2的接口
class OrbSLAMDriver: public ORB_SLAM2::System{
public:
    
    ///OrbSLAMDriver的构造函数
    OrbSLAMDriver(const string &strVocFile, const string &strSettingsFile, 
		  const eSensor sensor, const bool bUseViewer = true):
		  System(strVocFile, strSettingsFile, sensor, bUseViewer){
		    last_egomotion_->setIdentity();
		  };
		  
    virtual ~OrbSLAMDriver() {
      delete last_egomotion_;
    }
		  
private:
  Eigen::Matrix4f *last_egomotion_;
    
    
    
};
}//driver
#endif