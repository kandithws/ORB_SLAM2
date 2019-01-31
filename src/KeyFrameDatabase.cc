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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase (const std::shared_ptr<ORBVocabulary> voc):
    mpVoc(voc)
{
    mvInvertedFile.resize(voc->size());
}


void KeyFrameDatabase::add(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF);
}

void KeyFrameDatabase::erase(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        list<std::shared_ptr<KeyFrame> > &lKFs =   mvInvertedFile[vit->first];

        for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


vector<std::shared_ptr<KeyFrame> > KeyFrameDatabase::DetectLoopCandidates(std::shared_ptr<KeyFrame> pKF, float minScore)
{
    set<std::shared_ptr<KeyFrame> > spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<std::shared_ptr<KeyFrame> > lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<std::shared_ptr<KeyFrame> > &lKFs =   mvInvertedFile[vit->first];

            for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                std::shared_ptr<KeyFrame> pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<std::shared_ptr<KeyFrame> >();

    list<pair<float,std::shared_ptr<KeyFrame> > > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxstd::shared_ptr<CommonWords> 0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        std::shared_ptr<KeyFrame> pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<std::shared_ptr<KeyFrame> >();

    list<pair<float,std::shared_ptr<KeyFrame> > > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,std::shared_ptr<KeyFrame> > >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        std::shared_ptr<KeyFrame> pKFi = it->second;
        vector<std::shared_ptr<KeyFrame> > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        std::shared_ptr<KeyFrame> pBestKF = pKFi;
        for(vector<std::shared_ptr<KeyFrame> >::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            std::shared_ptr<KeyFrame> pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<std::shared_ptr<KeyFrame> > spAlreadyAddedKF;
    vector<std::shared_ptr<KeyFrame> > vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,std::shared_ptr<KeyFrame> > >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            std::shared_ptr<KeyFrame> pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

vector<std::shared_ptr<KeyFrame> > KeyFrameDatabase::DetectRelocalizationCandidates(std::shared_ptr<Frame> F)
{
    list<std::shared_ptr<KeyFrame> > lKFsSharingWords;

    // Search all keyframes that share a word with current frame
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            list<std::shared_ptr<KeyFrame> > &lKFs =   mvInvertedFile[vit->first];

            for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                std::shared_ptr<KeyFrame> pKFi=*lit;
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<std::shared_ptr<KeyFrame> >();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    list<pair<float,std::shared_ptr<KeyFrame> > > lScoreAndMatch;

    int nscores=0;

    // Compute similarity score.
    for(list<std::shared_ptr<KeyFrame> >::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        std::shared_ptr<KeyFrame> pKFi = *lit;

        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<std::shared_ptr<KeyFrame> >();

    list<pair<float,std::shared_ptr<KeyFrame> > > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(list<pair<float,std::shared_ptr<KeyFrame> > >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        std::shared_ptr<KeyFrame> pKFi = it->second;
        vector<std::shared_ptr<KeyFrame> > vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        std::shared_ptr<KeyFrame> pBestKF = pKFi;
        for(vector<std::shared_ptr<KeyFrame> >::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            std::shared_ptr<KeyFrame> pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;

            accScore+=pKF2->mRelocScore;
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    set<std::shared_ptr<KeyFrame> > spAlreadyAddedKF;
    vector<std::shared_ptr<KeyFrame> > vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,std::shared_ptr<KeyFrame> > >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            std::shared_ptr<KeyFrame> pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
