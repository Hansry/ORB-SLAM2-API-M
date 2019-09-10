#ifndef ORBSLAMDRIVER_H
#define ORBSLAMDRIVER_H

#include <string>
#include "thread"
#include <opencv2/core/core.hpp>
#include "System.h"
#include "MapDrawer.h"
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
		   last_egomotion_.setOnes();
		  };
		 
    virtual ~OrbSLAMDriver() {}
    
    ///@brief 返回世界坐标系到当前帧的变换Tcw
    Eigen::Matrix4f GetPose() const{
      return MatToEigen(this->GetWorldToCurrFramePose());
    }
    
    ///@brief 获取俩帧之前的相对位置
    void Track(){
      cv::Mat CurrentFrame = this->GetWorldToCurrFramePose();
      cv::Mat LastFrameInv = this->GetWorldTolastFramePose();
      (this->last_egomotion_) = MatToEigen(CurrentFrame*LastFrameInv);
    }
    
    Eigen::Matrix4f GetlastEgomotion() const{
      return last_egomotion_;
    }
    
    ORB_SLAM2::MapDrawer* GetMapDriver() const{
      return this->mpMapDrawer;
    }
    
    cv::Mat orbTrackStereo(cv::Mat& imleft, cv::Mat& imright, int timestamp) {
         return this->TrackStereo(imleft, imright, timestamp);
    }
    
    void orbShutdown(){
         this->Shutdown();
    }
    
    void orbSaveTrajectoryKITTI(string saveName){
       this->SaveTrajectoryKITTI(saveName);
    }
    
private:
    Eigen::Matrix4f last_egomotion_;    
};
}//driver
#endif