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
#include "utils/Config.h"

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
    GenerateColorMap(); // TODO -- read size from Label names!

    if (Config::getInstance().SystemParams().use_imu){
        mbUseIMU = true;
        auto tmpTcb = Config::getInstance().IMUParams().GetMatTcb();

        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);


        Rwc = tmpTcb.rowRange(0,3).colRange(0,3);
        twc = tmpTcb.rowRange(0,3).col(3);


        mTcb.m[0] = Rwc.at<float>(0,0);
        mTcb.m[1] = Rwc.at<float>(1,0);
        mTcb.m[2] = Rwc.at<float>(2,0);
        mTcb.m[3]  = 0.0;

        mTcb.m[4] = Rwc.at<float>(0,1);
        mTcb.m[5] = Rwc.at<float>(1,1);
        mTcb.m[6] = Rwc.at<float>(2,1);
        mTcb.m[7]  = 0.0;

        mTcb.m[8] = Rwc.at<float>(0,2);
        mTcb.m[9] = Rwc.at<float>(1,2);
        mTcb.m[10] = Rwc.at<float>(2,2);
        mTcb.m[11]  = 0.0;

        mTcb.m[12] = twc.at<float>(0);
        mTcb.m[13] = twc.at<float>(1);
        mTcb.m[14] = twc.at<float>(2);
        mTcb.m[15]  = 1.0;
    }
}

// From python's VOC Labelmap
void MapDrawer::GenerateColorMap(int N) {
    auto bitget = [](int &byteval, int idx){
        return ((byteval & (1 << idx)) != 0);
    };

    mLabelColorMap.clear();
    mLabelColorMap.resize(N);

    for (int i=0; i < N; i++){
        uint8_t r=0, g=0, b=0;
        int c = i;

        for (int j=0; j < 8; j++){
            r = r | ( bitget(c, 0) << (7-j) );
            g = g | ( bitget(c, 1) << (7-j) );
            b = b | ( bitget(c, 2) << (7-j) );
            c = c >> 3;
        }

        mLabelColorMap[i][0] = r / 255.0f;
        mLabelColorMap[i][1] = g / 255.0f;
        mLabelColorMap[i][2] = b / 255.0f;
    }
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
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

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
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
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
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

void MapDrawer::DrawObjects(const bool bDrawObj, const bool bDrawGraph) {
    auto vMapObjects = mpMap->GetAllMapObjects();

    if (bDrawObj){
        for (auto &pObj : vMapObjects){
            if(!pObj->IsReady())
                continue;

            cv::Mat Twc = pObj->GetPose().t();
            cv::Mat scale = pObj->GetScale();
            float x, y, z;
            x = scale.at<float>(0); y = scale.at<float>(1); z = scale.at<float>(2);
            float axis_scale = std::min(x , std::min(y,z)) * mMapObjectAxisScale;
            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            // Cuboid
            glLineWidth(mMapObjectLineWidth);
            auto& color = mLabelColorMap[pObj->mLabel];
            glColor3f(color[0],color[1],color[2]);
            // glColor3f(1.0f,0.0f,0.0f);
            glBegin(GL_LINES);

            glVertex3f(x,-y,-z);
            glVertex3f(x,y,-z);
            //2
            glVertex3f(x,y,-z);
            glVertex3f(x,y,z);
            //3
            glVertex3f(x,y,z);
            glVertex3f(x,-y,z);
            //4
            glVertex3f(x,-y,z);
            glVertex3f(x,-y,-z);
            //5
            glVertex3f(x,-y,-z);
            glVertex3f(-x,-y,-z);
            //6
            glVertex3f(-x,-y,-z);
            glVertex3f(-x,-y,z);
            //7
            glVertex3f(-x,-y,z);
            glVertex3f(x,-y,z);

            //8
            glVertex3f(x,y,-z);
            glVertex3f(-x,y,-z);
            //9
            glVertex3f(-x,y,-z);
            glVertex3f(-x,y,z);
            //10
            glVertex3f(-x,y,z);
            glVertex3f(x,y,z);
            //11
            glVertex3f(-x,-y,z);
            glVertex3f(-x,y,z);
            //12
            glVertex3f(-x,-y,-z);
            glVertex3f(-x,y,-z);

            glEnd();

            glLineWidth(mMapObjectAxisWidth);

            glBegin(GL_LINES);
            // x-axis
            glColor3f(1.0f, 0, 0);
            glVertex3f(0, 0, 0);
            glVertex3f(axis_scale, 0, 0);

            glColor3f(0, 1.0f, 0);
            glVertex3f(0, 0, 0);
            glVertex3f(0, axis_scale, 0);

            glColor3f(0.0,0.0,1.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0, axis_scale);
            glEnd();


            glPopMatrix();

            glColor3f(color[0],color[1],color[2]);
            glPointSize(mPointSize * 2.0);
            glBegin(GL_POINTS);
            for (auto& pMP : pObj->GetMapPoints()){
                cv::Mat pos = pMP->GetWorldPos();
                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            }

            glEnd();
        }
    }

    if (bDrawGraph){

        glLineWidth(mGraphLineWidth);
        glColor4f(0.75f,0.0f,0.75f, 0.6f);
        glBegin(GL_LINES);

        for (auto &pObj : vMapObjects){
            if(!pObj->IsReady())
                continue;

            cv::Mat Twc = pObj->GetPose();

            auto mapObservations = pObj->GetObservations();
            for (auto &key_val : mapObservations ){
                cv::Mat Owc = key_val.first->GetCameraCenter();
                glVertex3f(Owc.at<float>(0),Owc.at<float>(1),Owc.at<float>(2));
                glVertex3f(Twc.at<float>(0,3),Twc.at<float>(1,3),Twc.at<float>(2,3));
            }
        }
        glEnd();
    }

}

void MapDrawer::DrawGravity(){
    if (!mbGravityReady){
        // Try get gravity vector
        cv::Mat g;
        if (!mpMap->GetGravityVec(g))
            return;

        mbGravityReady = true;
        mGravityVec = g / cv::norm(g);
    }

    // Calculate gravity Vector

    glLineWidth(mGravityLineWidth);
    glColor4f(1.0f,0.0f,0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(
            mGravityVec.at<float>(0) * mGravityLineLength,
            mGravityVec.at<float>(1) * mGravityLineLength,
            mGravityVec.at<float>(2) * mGravityLineLength);
    glEnd();
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
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



    if (mbUseIMU) {
        // pangolin::OpenGlMatrix Twb = Twc * mTcb;
        glMultMatrixd(mTcb.m);
        glBegin(GL_LINES);
        // x-axis
        glColor3f(1.0f, 0, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(z, 0, 0);

        glColor3f(0, 1.0f, 0);
        glVertex3f(0, 0, 0);
        glVertex3f(0, z, 0);

        glColor3f(0.0,0.0,1.0f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, z);
        glEnd();
    }

    glPopMatrix();


}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

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
