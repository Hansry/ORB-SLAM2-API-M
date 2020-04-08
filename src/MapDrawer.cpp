/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

//关于gl相关的函数，可直接google, 并加上msdn关键词
void MapDrawer::DrawMapPoints()
{
    //取出所有的地图点
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    //取出mvpReferenceMapPoints，也即局部地图d点
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    //将vpRefMPs从vector容器类型转化为set容器类型，便于使用set::count快速统计
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    // for AllMapPoints
    //显示所有的地图点（不包括局部地图点），大小为2个像素，黑色
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    //glColor3f(0.0,0.0,0.0);
    glColor3f(1.0,0.0,0.0);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        // 不包括ReferenceMapPoints（局部地图点）
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    // for ReferenceMapPoints
    //显示局部地图点，大小为2个像素，红色
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    //glColor3f(0.0,0.0,0.0);
    glColor3f(1.0,1.0,1.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();
}

void MapDrawer::SetTracker(Tracking* Tracker){
     mpDrawerTracker = Tracker;
}


void MapDrawer::DrawTracjectory(){ 
   const float  &w = mKeyFrameSize;
   const float h = w*0.35;
   const float z = w*0.3;
   
   vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();   
   sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
   
   float last_x = 0.0;
   float last_y = 0.0;
   float last_z = 0.0;
   //第一帧为世界坐标系
   cv::Mat Two = vpKFs[0]->GetPoseInverse();
   list<ORB_SLAM2::KeyFrame*>::iterator lRit =  mpDrawerTracker->mlpReferences.begin();
   list<double>::iterator lT = mpDrawerTracker->mlFrameTimes.begin();
   for(list<cv::Mat>::iterator lit=mpDrawerTracker->mlRelativeFramePoses.begin(),
        lend=mpDrawerTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++){
      
      ORB_SLAM2::KeyFrame* pKF = *lRit;
      cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
      
      while(pKF->isBad()){
	Trw = Trw*pKF->mTcp;
	pKF = pKF->GetParent();
      }
      //其中Trw为世界坐标系到参考帧坐标系的相对变换
      Trw = Trw * pKF->GetPose()*Two;
      cv::Mat Tcw = (*lit) * Trw; 
      
      
      cv::Mat Twc = cv::Mat::eye(4,4,CV_32F);
      cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
      cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
      
      for(int i=0;i<3;i++){
	for(int j=0;j<3;j++){
	  Twc.at<float>(i,j) = Rwc.at<float>(i,j);
	}
      }
      Twc.at<float>(0,3) = twc.at<float>(0);
      Twc.at<float>(1,3) = twc.at<float>(1);
      Twc.at<float>(2,3) = twc.at<float>(2);
      
      Twc = Twc.t();
                
      glPushMatrix();

      //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
      //因为OpenGL中的矩阵为列优先存储
      glMultMatrixf(Twc.ptr<GLfloat>(0));

      //设置绘制图形时线的宽度
      glLineWidth(mKeyFrameLineWidth);
      //设置当前颜色为蓝色(关键帧图标显示为蓝色)
      glColor3f(0.0f,0.0f,1.0f);
      //用线将下面的顶点两两相连
      glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(w,h,z);
      glVertex3f(0,0,0);
      glVertex3f(w,-h,z);
      glVertex3f(0,0,0);
      glVertex3f(-w,-h,z);
      glVertex3f(0,0,0);
      glVertex3f(-w,h,z);

      glVertex3f(w,h,z);
      glVertex3f(w,-h,z);

      glVertex3f(-w,h,z);
      glVertex3f(-w,-h,z);

      glVertex3f(-w,h,z);
      glVertex3f(w,h,z);

      glVertex3f(-w,-h,z);
      glVertex3f(w,-h,z);
      glEnd();

      glPopMatrix();  
      
      if(lit!=mpDrawerTracker->mlRelativeFramePoses.begin()){
          //设置绘制图形时线的宽度
          glLineWidth(2.0);
          //设置共视图连接线为绿色，透明度为0.6f
          glColor4f(0.0f,1.0f,0.0f,0.6f);
          glBegin(GL_LINES);
          glVertex3f(last_x,last_y,last_z);
          glVertex3f(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
          glEnd();
      }
      last_x = twc.at<float>(0);
      last_y = twc.at<float>(1);
      last_z = twc.at<float>(2);
  }
}

//关于gl相关的函数，可直接google, 并加上msdn关键词
void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    //历史关键帧图标：宽度占总宽度比例为0.05
    const float &w = mKeyFrameSize;
    const float h = w*0.4;
    const float z = w*0.6;

    //步骤1：取出所有的关键帧
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    //步骤2：显示所有关键帧图标
    //通过显示界面选择是否显示历史关键帧图标
    if(bDrawKF)
    {
        for(int i=startKeyframe; i < vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            //转置, OpenGL中的矩阵为列优先存储
            cv::Mat Twc = pKF->GetPoseInverse().t();
	    
            glPushMatrix();

            //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
            //因为OpenGL中的矩阵为列优先存储
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            //设置绘制图形时线的宽度
            glLineWidth(mKeyFrameLineWidth);
            //设置当前颜色为蓝色(关键帧图标显示为蓝色)
            glColor3f(0.0f,0.0f,1.0f);
            //用线将下面的顶点两两相连
	    glBegin(GL_LINES);
	    glVertex3f(0,0,0);
	    glVertex3f(w,h,z);
	    glVertex3f(0,0,0);
	    glVertex3f(w,-h,z);
	    glVertex3f(0,0,0);
	    glVertex3f(-w,-h,z);
	    glVertex3f(0,0,0);
	    glVertex3f(-w,h,z);

	    glVertex3f(w,h,z);
	    glVertex3f(w,-h,z);

	    glVertex3f(-w,h,z);
	    glVertex3f(-w,-h,z);

	    glVertex3f(-w,h,z);
	    glVertex3f(w,h,z);

	    glVertex3f(-w,-h,z);
	    glVertex3f(w,-h,z);
	    glEnd();
            glPopMatrix();
        }
    }

    //步骤3：显示所有关键帧的Essential Graph
    //通过显示界面选择是否显示关键帧连接关系
    if(bDrawGraph)
    {
        //设置绘制图形时线的宽度
        glLineWidth(mGraphLineWidth);
        //设置共视图连接线为绿色，透明度为0.6f
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            //步骤3.1 共视程度比较高的共视关键帧用线连接
            //遍历每一个关键帧，得到它们共视程度比较高的关键帧
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            //遍历每一个关键帧，得到它在世界坐标系下的相机坐标
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            //步骤3.2 连接最小生成树
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            //步骤3.3 连接闭环时形成的连接关系
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

//关于gl相关的函数，可直接google, 并加上msdn关键词
void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    //相机模型大小：宽度占总宽度比例为0.08
    const float &w = mCameraSize;
    const float h = w*0.4;
    const float z = w*0.6;

    //百度搜索：glPushMatrix 百度百科
    glPushMatrix();

    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    //设置绘制图形时线的宽度
    glLineWidth(mCameraLineWidth);
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f,1.0f,0.0f);
    //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

cv::Mat MapDrawer::GetCameraPose(){
    unique_lock<mutex> lock(mMutexCamera);
    return mCameraPose;
}

// 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
