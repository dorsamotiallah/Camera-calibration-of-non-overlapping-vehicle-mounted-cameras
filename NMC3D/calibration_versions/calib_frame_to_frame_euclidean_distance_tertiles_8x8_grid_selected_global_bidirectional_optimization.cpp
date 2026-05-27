#include "Sim3Solver.h"
#include "calib.hpp"
#include "edge.hpp"

#include <algorithm>
#include <array>
#include <cmath>

using std::cout;
using std::endl;
using namespace ORB_SLAM3;

const float todeg = 180 / M_PI;

namespace
{

struct GlobalDepthMatchRef
{
    size_t kfIdx;
    size_t matchIdx;
    float depth;
};

constexpr int kSelectionGridRows = 8;
constexpr int kSelectionGridCols = 8;
constexpr int kSelectionGridCells = kSelectionGridRows * kSelectionGridCols;

int CountMatches(const vector<MapPoint*>& vpMatches)
{
    int nMatches = 0;
    for(MapPoint* pMP : vpMatches)
    {
        if(pMP && !pMP->isBad())
            nMatches++;
    }
    return nMatches;
}

int CountMatches(const vector<vector<MapPoint*>>& vvpMatches)
{
    int nMatches = 0;
    for(const vector<MapPoint*>& vpMatches : vvpMatches)
        nMatches += CountMatches(vpMatches);
    return nMatches;
}

float PercentileDepth(const vector<GlobalDepthMatchRef>& vDepthRefs, const float percentile)
{
    if(vDepthRefs.empty())
        return 0.0f;

    const float clamped = std::max(0.0f, std::min(1.0f, percentile));
    const size_t idx = std::min(static_cast<size_t>(clamped * static_cast<float>(vDepthRefs.size() - 1)), vDepthRefs.size() - 1);
    return vDepthRefs[idx].depth;
}

int GridCellForKeyPoint(KeyFrame* pKF, const size_t matchIdx)
{
    const cv::KeyPoint& kp = pKF->mvKeysUn[matchIdx];
    const float width = std::max(1.0f, static_cast<float>(pKF->mnMaxX - pKF->mnMinX));
    const float height = std::max(1.0f, static_cast<float>(pKF->mnMaxY - pKF->mnMinY));

    int col = static_cast<int>(kSelectionGridCols * (kp.pt.x - static_cast<float>(pKF->mnMinX)) / width);
    int row = static_cast<int>(kSelectionGridRows * (kp.pt.y - static_cast<float>(pKF->mnMinY)) / height);

    col = std::max(0, std::min(kSelectionGridCols - 1, col));
    row = std::max(0, std::min(kSelectionGridRows - 1, row));
    return row * kSelectionGridCols + col;
}

template<typename TDepthRef>
size_t CountOccupiedGridCells(const std::array<vector<TDepthRef>, kSelectionGridCells>& vCells)
{
    size_t nOccupied = 0;
    for(const vector<TDepthRef>& vCell : vCells)
    {
        if(!vCell.empty())
            nOccupied++;
    }
    return nOccupied;
}

size_t SelectGlobalGridDistributedRefs(const vector<KeyFrame*>& vpKF1s, const vector<GlobalDepthMatchRef>& vGroup,
                                       const size_t nTarget, const vector<vector<MapPoint*>>& vvpMatches,
                                       vector<vector<MapPoint*>>& vvpSelectedMatches, size_t& nOccupiedCells)
{
    std::array<vector<GlobalDepthMatchRef>, kSelectionGridCells> vCells;

    for(const GlobalDepthMatchRef& ref : vGroup)
        vCells[GridCellForKeyPoint(vpKF1s[ref.kfIdx], ref.matchIdx)].push_back(ref);

    nOccupiedCells = CountOccupiedGridCells(vCells);

    for(vector<GlobalDepthMatchRef>& vCell : vCells)
    {
        std::sort(vCell.begin(), vCell.end(), [&vpKF1s](const GlobalDepthMatchRef& a, const GlobalDepthMatchRef& b) {
            const cv::KeyPoint& kpa = vpKF1s[a.kfIdx]->mvKeysUn[a.matchIdx];
            const cv::KeyPoint& kpb = vpKF1s[b.kfIdx]->mvKeysUn[b.matchIdx];
            if(kpa.response != kpb.response)
                return kpa.response > kpb.response;
            if(kpa.octave != kpb.octave)
                return kpa.octave < kpb.octave;
            return a.depth < b.depth;
        });
    }

    std::array<size_t, kSelectionGridCells> vCursor;
    vCursor.fill(0);

    size_t nSelected = 0;
    bool bProgress = true;
    while(nSelected < nTarget && bProgress)
    {
        bProgress = false;
        for(size_t iCell = 0; iCell < vCells.size() && nSelected < nTarget; iCell++)
        {
            vector<GlobalDepthMatchRef>& vCell = vCells[iCell];
            if(vCursor[iCell] >= vCell.size())
                continue;

            const GlobalDepthMatchRef& ref = vCell[vCursor[iCell]];
            vvpSelectedMatches[ref.kfIdx][ref.matchIdx] = vvpMatches[ref.kfIdx][ref.matchIdx];
            vCursor[iCell]++;
            nSelected++;
            bProgress = true;
        }
    }

    return nSelected;
}

size_t SelectFrameGridDistributedRefs(KeyFrame* pKF1, const vector<GlobalDepthMatchRef>& vGroup,
                                      const size_t nTarget, const vector<MapPoint*>& vpMatches,
                                      vector<MapPoint*>& vpSelectedMatches, size_t& nOccupiedCells)
{
    std::array<vector<GlobalDepthMatchRef>, kSelectionGridCells> vCells;

    for(const GlobalDepthMatchRef& ref : vGroup)
        vCells[GridCellForKeyPoint(pKF1, ref.matchIdx)].push_back(ref);

    nOccupiedCells = CountOccupiedGridCells(vCells);

    for(vector<GlobalDepthMatchRef>& vCell : vCells)
    {
        std::sort(vCell.begin(), vCell.end(), [pKF1](const GlobalDepthMatchRef& a, const GlobalDepthMatchRef& b) {
            const cv::KeyPoint& kpa = pKF1->mvKeysUn[a.matchIdx];
            const cv::KeyPoint& kpb = pKF1->mvKeysUn[b.matchIdx];
            if(kpa.response != kpb.response)
                return kpa.response > kpb.response;
            if(kpa.octave != kpb.octave)
                return kpa.octave < kpb.octave;
            return a.depth < b.depth;
        });
    }

    std::array<size_t, kSelectionGridCells> vCursor;
    vCursor.fill(0);

    size_t nSelected = 0;
    bool bProgress = true;
    while(nSelected < nTarget && bProgress)
    {
        bProgress = false;
        for(size_t iCell = 0; iCell < vCells.size() && nSelected < nTarget; iCell++)
        {
            vector<GlobalDepthMatchRef>& vCell = vCells[iCell];
            if(vCursor[iCell] >= vCell.size())
                continue;

            const GlobalDepthMatchRef& ref = vCell[vCursor[iCell]];
            vpSelectedMatches[ref.matchIdx] = vpMatches[ref.matchIdx];
            vCursor[iCell]++;
            nSelected++;
            bProgress = true;
        }
    }

    return nSelected;
}

}

CalibC2C::CalibC2C(System* src, System* dst)
{
    // get Atlas
    SubSystem* ssrc = static_cast<SubSystem*>(src);
    SubSystem* sdst = static_cast<SubSystem*>(dst);
    
    auto srcAtlas = static_cast<SubAtlas*>(ssrc->getAtlas());
    auto dstAtlas = static_cast<SubAtlas*>(sdst->getAtlas());
    srcAtlas->setFirstCurrentMap();
    dstAtlas->setFirstCurrentMap();
    srcKFs = srcAtlas->GetAllKeyFrames();
    dstKFs = dstAtlas->GetAllKeyFrames();

    mpKeyFrameDB = static_cast<SubKeyFrameDB*>(sdst->getKeyFrameDatabase());

    matcherBoW = new ORBmatcher(0.9, true);
    matcher = new ORBmatcher(0.75, true);

    mpLC = new SubLoopClosing();
}

vector<KeyFrame*> CalibC2C::DetectNBestCandidates(KeyFrame* pKF, int N)
{
    vector<KeyFrame*> res;
    // mpKeyFrameBD should be target kfs database
    auto invertedFile = mpKeyFrameDB->getInvertedFile();
    auto orbvoc = mpKeyFrameDB->getVocabulary();

    list<KeyFrame*> lKFsSharingWords;
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        list<KeyFrame*> &lKFs = invertedFile[vit->first];

        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            KeyFrame* pKFi = *lit;
            
            // not place rec query before
            if(pKFi->mnPlaceRecognitionQuery != pKF->mnId)
            {
                pKFi->mnPlaceRecognitionWords = 0;
                pKFi->mnPlaceRecognitionQuery = pKF->mnId;
                lKFsSharingWords.push_back(pKFi);
            }
            pKFi->mnPlaceRecognitionWords++;
        }
    }

    if(lKFsSharingWords.empty())    return res;

    cout << "share same words kf size: " << lKFsSharingWords.size() << endl;
    // find threshold and filter them
    auto it = std::max_element(lKFsSharingWords.begin(), lKFsSharingWords.end(), [](KeyFrame* a, KeyFrame* b){
        return a->mnPlaceRecognitionWords < b->mnPlaceRecognitionWords;
    });
    int minCommonWords = (*it)->mnPlaceRecognitionWords * 0.5f;

    // filter
    list<pair<float,KeyFrame*> > lScoreAndMatch;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend=lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->mnPlaceRecognitionWords > minCommonWords)
        {
            float si = orbvoc->score(pKF->mBowVec, pKFi->mBowVec);
            pKFi->mPlaceRecognitionScore = si;
            lScoreAndMatch.push_back(make_pair(si, pKFi));
        }
    }
    if(lScoreAndMatch.empty())  return res;

    cout << "after filter words less than "
    << minCommonWords << endl
    << "size: " << lScoreAndMatch.size() << endl;

    // Lets now accumulate score by covisibility
    list<pair<float, KeyFrame*>> lAccScoreAndMatch;
    float bestAccScore = 0;
    for(auto it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFrame* pBestKF = pKFi;

        for(auto vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnPlaceRecognitionQuery == pKF->mnId)
            {
                accScore += pKF2->mPlaceRecognitionScore;
                // find best score kf in covis kfs of this kf
                if(pKF2->mPlaceRecognitionScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mPlaceRecognitionScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
        if(accScore > bestAccScore)   bestAccScore = accScore;
    }

    cout << "find best covis group size: " << lAccScoreAndMatch.size() << endl;

    lAccScoreAndMatch.sort([](const pair<float, KeyFrame*>& a, const pair<float, KeyFrame*>& b){
        return a.first > b.first;
    });
    
    // remove duplcated kf from best group
    set<KeyFrame*> duplicatedKF;
    for(auto it=lAccScoreAndMatch.begin(); it!=lAccScoreAndMatch.end(); it++)
    {
        KeyFrame* pKFi = it->second;
        if(pKFi->isBad())   continue;

        if(duplicatedKF.count(pKFi) == 0){
            duplicatedKF.insert(pKFi);
            res.push_back(pKFi);

            if(res.size() >= N) break;
        }
    }

    return res;
}

// vvMPs is expressed at frame of Cand
bool CalibC2C::DetectCommonRegionsFromCand(KeyFrame* pKF, vector<KeyFrame*>& vpCand, KeyFrame* &pMatchedKF2, g2o::Sim3 &g2oScw,
        int& bestMatchesReprojNum, vector<MapPoint*> &vpMPs, vector<MapPoint*> &vpMatchedMPs)
{
    KeyFrame* pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    vector<MapPoint*> vpBestMapPoints;
    vector<MapPoint*> vpBestMatchedMapPoints;

    for(KeyFrame* pKFi : vpCand)
    {
        if(!pKFi || pKFi->isBad())  continue;

        // 1. match pKF with cov kf of pKFi in Cand, find the best one
        // searchByBoW, brute-force search from two kf
        auto vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(5);
        vector<vector<MapPoint*>> vvpMatchedMPs(vpCovKFi.size());
        int nMostBoWNumMatches = 0;
        KeyFrame* pMostBoWMatchesKF = nullptr;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())    continue;

            int num = matcherBoW->SearchByBoW(pKF, vpCovKFi[j], vvpMatchedMPs[j]);
            if(num > nMostBoWNumMatches){
                nMostBoWNumMatches = num;
                pMostBoWMatchesKF = vpCovKFi[j];
            }
        }

        // 2. find all mp belongs to pKFi and its covis and pKF itself
        // make optimizeSim3 interface
        auto nMPNums = pKF->GetMapPointMatches().size();
        set<MapPoint*> duplicateMP;
        vector<MapPoint*> vpMatchedPoints(nMPNums, nullptr);        // mp
        vector<KeyFrame*> vpKeyFrameMatchedMP(nMPNums, nullptr);    // mp to kf
        int numBoWMatches = 0;
        for(int j=0; j<vpCovKFi.size(); j++){
            for(int k=0; k<vvpMatchedMPs[j].size(); k++)
            {
                MapPoint* mp = vvpMatchedMPs[j][k];
                if(!mp || mp->isBad())  continue;

                if(duplicateMP.count(mp) == 0){
                    duplicateMP.insert(mp);
                    numBoWMatches++;
                    vpMatchedPoints[k] = mp;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }
        cout << "search by bow get matched mp size: " << numBoWMatches << endl;

        // 3. solve sim3
        if(numBoWMatches >= 20){
            bool bFixedScale = false;
            int nBoWInliers = 8;
            Sim3Solver solver(pKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300);
        
            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            cv::Mat mTcm;
            while(!bConverge && !bNoMore)
            {
                mTcm = Converter::toCvMat(solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge));
            }

            if(bConverge)
            {
                cout << "solve sim3 converged!!" << endl;
                // 4. if sim3 is solved normally, prepare all mp from covis of MostBoWMatchedKF
                // searchByProjection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(5);

                duplicateMP.clear();
                vector<MapPoint*> vpMapPoints;
                vector<KeyFrame*> vpKeyFrames;
                for(KeyFrame* pCovKFi : vpCovKFi)
                {
                    for(MapPoint* mp : pCovKFi->GetMapPointMatches())
                    {
                        if(!mp || mp->isBad())  continue;
                        if(duplicateMP.count(mp) == 0){
                            duplicateMP.insert(mp);
                            vpMapPoints.push_back(mp);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }

                g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(), solver.GetEstimatedTranslation().cast<double>(), solver.GetEstimatedScale());
                g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(), pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
                Sophus::Sim3<float> mScw = Converter::toSophus(gScw);
                
                vpMatchedMPs.assign(nMPNums, static_cast<MapPoint*>(nullptr));
                vpKeyFrameMatchedMP.assign(nMPNums, static_cast<KeyFrame*>(nullptr));

                int numProjMatches = matcher->SearchByProjection(pKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMPs, vpKeyFrameMatchedMP, 8, 1.5);
                cout << "search by projection get size: " << numProjMatches << endl;

                // 5. after use searchByProjection to get more mp, optimize Scm with them
                if(numProjMatches >= 25)
                {
                    Eigen::Matrix<double, 7, 7> mHessian7x7;
                    int numOptMatches = Optimizer::OptimizeSim3(pKF, pKFi, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);
                    cout << "optim sim3 get size: " << numOptMatches << endl;

                    // 6. when optimed Scm is found, search by projection again to get more matched points 
                    if(numOptMatches >= 10)
                    {
                        gScw = gScm * gSmw;
                        vpMatchedMPs.assign(nMPNums, static_cast<MapPoint*>(nullptr));
                        // shrink search range
                        numProjMatches = matcher->SearchByProjection(pKF, mScw, vpMapPoints, vpMatchedMPs, 5, 2.0);
                        cout << "after optim sim3 search by projection get size: " << numProjMatches << endl;

                        if(numProjMatches >= 40)
                        {
                            // 7. geometry validation: use covis kf of pKF to detect common regions of MostBoWMatchKF
                            vector<KeyFrame*> vpCurrentCovKFs = pKF->GetBestCovisibilityKeyFrames(5);

                            int validKF = 0;
                            for(KeyFrame* pKFj : vpCurrentCovKFs)
                            {
                                Eigen::Matrix4d mTjc = (pKFj->GetPose() * pKF->GetPoseInverse()).matrix().cast<double>();
                                g2o::Sim3 gSjc(mTjc.topLeftCorner<3, 3>(), mTjc.topRightCorner<3, 1>(), 1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPoint*> vpMatchedMPs_j;
                                bool bValid = mpLC->SubDetectCommonRegionsFromLastKF(pKFj, pMostBoWMatchesKF, gSjw, numProjMatches_j, vpMapPoints, vpMatchedMPs_j);
                                if(bValid)  validKF++;
                            }
                            cout << "geometry validation pass kf size: " << validKF << endl;

                            // 8. balance matches by local frame-to-frame depth before saving/scoring this candidate
                            vector<MapPoint*> vpBalancedMatchedMPs = SelectFrameDepthBalancedMatches(pKF, pMostBoWMatchesKF, vpMatchedMPs);
                            const int numBalancedMatches = CountMatches(vpBalancedMatchedMPs);
                            if(nBestMatchesReproj < numBalancedMatches)
                            {
                                nBestMatchesReproj = numBalancedMatches; 
                                nBestNumCoindicendes = validKF;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpBalancedMatchedMPs;
                            }
                        }
                    }
                }
            }
        }
    }

    if(nBestMatchesReproj)
    {
        bestMatchesReprojNum = nBestMatchesReproj;
        pMatchedKF2 = pBestMatchedKF;
        g2oScw = g2oBestScw;
        vpMPs = vpBestMapPoints;
        vpMatchedMPs = vpBestMatchedMapPoints;
        return nBestNumCoindicendes >= 1;
    }

    return false;
}

// vvMPs is expressed at frame of Cand
// finalPose1 is camera1's final pose expressed at start frame
int CalibC2C::OptimizeSim3ForCalibr(const vector<KeyFrame*>& vpKF1s, const vector<KeyFrame*>& vpKF2s, vector<vector<MapPoint*>>& vvpMatches,
    g2o::Sim3 &g2oS12, const float th2, const bool bFixScale, Eigen::Isometry3f finalPose1, Eigen::Isometry3f finalPose2)
{
    // 1. init g2o optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);
    const float deltaHuber = sqrt(th2);

    // camera instrincs
    KeyFrame* KF1 = vpKF1s.front();
    KeyFrame* KF2 = vpKF2s.front();

    // 2. set vertex of extrinsic
    g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale = bFixScale;
    vSim3->setFixed(false);                          
    vSim3->_principle_point1[0] = KF1->cx;
    vSim3->_principle_point1[1] = KF1->cy;
    vSim3->_focal_length1[0] = KF1->fx;  
    vSim3->_focal_length1[1] = KF1->fy; 
    vSim3->_principle_point2[0] = KF2->cx;
    vSim3->_principle_point2[1] = KF2->cy;
    vSim3->_focal_length2[0] = KF2->fx;
    vSim3->_focal_length2[1] = KF2->fy;

    vSim3->setId(0);
    optimizer.addVertex(vSim3);

    // 3. set vertex of map points
    int nKF = vpKF1s.size();
    int nCorrespondences = 0;
    int id1 = -1;
    int id2 = 0;
    vector<vector<g2o::EdgeSim3ProjectXYZForCalibr*>> vvpEdges12(nKF);
    vector<vector<g2o::EdgeInverseSim3ProjectXYZForCalibr*>> vvpEdges21(nKF);

    for(int idx=0; idx<nKF; idx++)
    {
        KeyFrame *pKF1 = vpKF1s[idx];
        KeyFrame *pKF2 = vpKF2s[idx];
        const vector<MapPoint*>& vpMapPoints = pKF1->GetMapPointMatches();
        const vector<MapPoint*>& vpMatches = vvpMatches[idx];

        int N = vpMatches.size();
        vector<g2o::EdgeSim3ProjectXYZForCalibr*> vpEdges12;       
        vector<g2o::EdgeInverseSim3ProjectXYZForCalibr*> vpEdges21;
        vpEdges12.reserve(N);
        vpEdges21.reserve(N);

        Eigen::Matrix3d R1w, R2w;
        Eigen::Vector3d t1w, t2w;
        Eigen::Isometry3f P1w, P2w;
        P1w.matrix() = pKF1->GetPose().matrix() * finalPose1.matrix();
        P2w.matrix() = pKF2->GetPose().matrix() * finalPose2.matrix();
        R1w = P1w.rotation().cast<double>();
        R2w = P2w.rotation().cast<double>();
        t1w = P1w.translation().cast<double>();
        t2w = P2w.translation().cast<double>();

        for(int i=0; i<N; i++){
            if(!vpMatches[i])   continue;

            auto pMP1 = vpMapPoints[i];
            auto pMP2 = vpMatches[i];
            
            const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));
            if(pMP1 == nullptr || pMP2 == nullptr || pMP1->isBad() || pMP2->isBad() || i2 < 0)
                continue;
            
            id1 += 2;
            id2 += 2;
            nCorrespondences++;

            // mp see from camera 1
            g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
            Eigen::Vector3f P3D1w = pMP1->GetWorldPos();
            // trans to final if need
            P3D1w = finalPose1.inverse() * P3D1w;
            vPoint1->setEstimate(P3D1w.cast<double>());
            vPoint1->setId(id1);
            vPoint1->setFixed(true);
            optimizer.addVertex(vPoint1);

            // mp see from camera 2
            g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
            Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
            P3D2w = finalPose2.inverse() * P3D2w;
            vPoint2->setEstimate(P3D2w.cast<double>());
            vPoint2->setId(id2);
            vPoint2->setFixed(true);
            optimizer.addVertex(vPoint2);

            // link edge
            // 1. mp from 2 project to 1 and make residual with mp from 1
            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            g2o::EdgeSim3ProjectXYZForCalibr *e12 = new g2o::EdgeSim3ProjectXYZForCalibr(R1w, t1w);
            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

            g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);
            
            // 2. mp from 1 project to 2 and make residual with mp from 2 
            Eigen::Matrix<double, 2, 1> obs2;
            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;

            g2o::EdgeInverseSim3ProjectXYZForCalibr *e21 = new g2o::EdgeInverseSim3ProjectXYZForCalibr(R2w, t2w);
            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

            g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
        }

        vvpEdges12[idx] = vpEdges12;
        vvpEdges21[idx] = vpEdges21;
    }

    // 4. start optimize
    int outliers;
    for(int i=0; i<4; i++)
    {
        vSim3->setEstimate(g2oS12);
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

        outliers = 0;
        for(int j=0; j<nKF; j++){
            auto& vpEdges12 = vvpEdges12[j];
            auto& vpEdges21 = vvpEdges21[j];

            for(int k=0; k<vpEdges12.size(); k++){
                auto e12 = vpEdges12[k];
                auto e21 = vpEdges21[k];

                if (!e12 || !e21)   continue;

                if (e12->chi2() > th2 || e21->chi2() > th2)
                {
                    e12->setLevel(1);
                    e21->setLevel(1);
                    outliers++;
                } else
                {
                    e12->setLevel(0);
                    e21->setLevel(0);
                }
            }
        }
    }

    // calc reproject error
    double SumChi2 = 0;
    for(int j=0; j<nKF; j++)
    {
        auto& vpEdges12 = vvpEdges12[j];
        auto& vpEdges21 = vvpEdges21[j];

        for(int k=0; k<vpEdges12.size(); k++)
        {
            auto e12 = vpEdges12[k];
            auto e21 = vpEdges21[k];

            if (!e12 || !e21)   continue;

            if (e12->chi2() <= th2 && e21->chi2() <= th2)
                SumChi2 += sqrt(abs(e12->chi2())) + sqrt(abs(e21->chi2()));
        }
    }

    SumChi2 /= (2 * (nCorrespondences - outliers));
    g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
    g2oS12 = vSim3_recov->estimate();

    return nCorrespondences - outliers;
}

void printResult(const g2o::Sim3& pose, const string& header)
{
    vector<float> euler = Converter::toEuler(Converter::toCvMat(pose.rotation().toRotationMatrix()));
    cout << header << endl;
    cout << "euler: ";
    for(auto&& e : euler)   cout << e * todeg << " ";
    cout << endl << "trans: ";
    Eigen::Vector3d t = pose.translation();
    for(int i=0; i<3; i++)  cout << t(i) << " ";
    cout << endl;
}

vector<MapPoint*> CalibC2C::SelectFrameDepthBalancedMatches(KeyFrame* pKF1, KeyFrame* pKF2, const vector<MapPoint*>& vpMatches)
{
    vector<MapPoint*> vpSelectedMatches(vpMatches.size(), nullptr);
    vector<GlobalDepthMatchRef> vDepthRefs;

    if(!bUseDepthBalancedSelection || !pKF1 || !pKF2 || pKF1->isBad() || pKF2->isBad())
        return vpMatches;

    const vector<MapPoint*>& vpMapPoints = pKF1->GetMapPointMatches();
    if(vpMapPoints.size() != vpMatches.size())
        return vpMatches;

    const Eigen::Matrix3f Rcw = pKF1->GetRotation();
    const Eigen::Vector3f tcw = pKF1->GetTranslation();

    for(size_t i = 0; i < vpMatches.size(); i++)
    {
        MapPoint* pMP1 = vpMapPoints[i];
        MapPoint* pMP2 = vpMatches[i];
        if(!pMP1 || !pMP2 || pMP1->isBad() || pMP2->isBad())
            continue;

        const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));
        if(i2 < 0)
            continue;

        const Eigen::Vector3f Pc1 = Rcw * pMP1->GetWorldPos() + tcw;
        const float distance = Pc1.norm();
        if(!std::isfinite(distance) || distance <= 0.0f)
            continue;

        vDepthRefs.push_back({0, i, distance});
    }

    if(vDepthRefs.size() < static_cast<size_t>(minDepthBalancedMatches))
    {
        cout << "frame-to-frame depth selection skipped: valid " << vDepthRefs.size() << endl;
        return vpMatches;
    }

    std::sort(vDepthRefs.begin(), vDepthRefs.end(), [](const GlobalDepthMatchRef& a, const GlobalDepthMatchRef& b) {
        return a.depth < b.depth;
    });

    const size_t nTotal = vDepthRefs.size();
    const size_t nNearEnd = nTotal / 3;
    const size_t nFarBegin = (2 * nTotal) / 3;
    const float nearThreshold = vDepthRefs[nNearEnd - 1].depth;
    const float farThreshold = vDepthRefs[nFarBegin].depth;

    std::array<vector<GlobalDepthMatchRef>, 3> vDepthGroups;
    vDepthGroups[0].assign(vDepthRefs.begin(), vDepthRefs.begin() + nNearEnd);
    vDepthGroups[1].assign(vDepthRefs.begin() + nNearEnd, vDepthRefs.begin() + nFarBegin);
    vDepthGroups[2].assign(vDepthRefs.begin() + nFarBegin, vDepthRefs.end());

    const size_t nNear = vDepthGroups[0].size();
    const size_t nMiddle = vDepthGroups[1].size();
    const size_t nFar = vDepthGroups[2].size();
    const size_t nPerGroup = std::min(nNear, std::min(nMiddle, nFar));

    if(nPerGroup == 0 || 3 * nPerGroup < static_cast<size_t>(minDepthBalancedMatches))
    {
        cout << "frame-to-frame depth selection skipped: near " << nNear
             << ", middle " << nMiddle
             << ", far " << nFar << endl;
        return vpMatches;
    }

    std::array<size_t, 3> vOccupiedCells;
    vOccupiedCells.fill(0);
    std::array<size_t, 3> vSelectedPerDepth;
    vSelectedPerDepth.fill(0);

    for(size_t iGroup = 0; iGroup < vDepthGroups.size(); iGroup++)
    {
        vSelectedPerDepth[iGroup] = SelectFrameGridDistributedRefs(pKF1, vDepthGroups[iGroup], nPerGroup,
                                                                   vpMatches, vpSelectedMatches,
                                                                   vOccupiedCells[iGroup]);
    }

    cout << "frame-to-frame depth selection: valid " << nTotal
         << ", near " << nNear
         << ", middle " << nMiddle
         << ", far " << nFar
         << ", selected per group " << nPerGroup
         << ", selected " << CountMatches(vpSelectedMatches) << endl;
    cout << "frame-to-frame 8x8 grid cells occupied: near " << vOccupiedCells[0]
         << ", middle " << vOccupiedCells[1]
         << ", far " << vOccupiedCells[2]
         << "; selected near " << vSelectedPerDepth[0]
         << ", middle " << vSelectedPerDepth[1]
         << ", far " << vSelectedPerDepth[2] << endl;
    cout << "frame-to-frame 3d distance stats: min " << PercentileDepth(vDepthRefs, 0.0f)
         << ", near threshold " << nearThreshold
         << ", median " << PercentileDepth(vDepthRefs, 0.50f)
         << ", far threshold " << farThreshold
         << ", max " << PercentileDepth(vDepthRefs, 1.0f) << endl;

    return vpSelectedMatches;
}

vector<vector<MapPoint*>> CalibC2C::SelectGlobalDepthBalancedMatches(const vector<KeyFrame*>& vpKF1s, const vector<KeyFrame*>& vpKF2s,
    const vector<vector<MapPoint*>>& vvpMatches)
{
    vector<vector<MapPoint*>> vvpSelectedMatches(vvpMatches.size());
    vector<GlobalDepthMatchRef> vDepthRefs;

    for(size_t idx = 0; idx < vvpMatches.size(); idx++)
    {
        KeyFrame* pKF1 = vpKF1s[idx];
        KeyFrame* pKF2 = vpKF2s[idx];
        if(!pKF1 || !pKF2 || pKF1->isBad() || pKF2->isBad())
            continue;

        const vector<MapPoint*>& vpMapPoints = pKF1->GetMapPointMatches();
        const vector<MapPoint*>& vpMatches = vvpMatches[idx];
        vvpSelectedMatches[idx] = vector<MapPoint*>(vpMatches.size(), nullptr);

        if(vpMapPoints.size() != vpMatches.size())
            continue;

        const Eigen::Matrix3f Rcw = pKF1->GetRotation();
        const Eigen::Vector3f tcw = pKF1->GetTranslation();

        for(size_t i = 0; i < vpMatches.size(); i++)
        {
            MapPoint* pMP1 = vpMapPoints[i];
            MapPoint* pMP2 = vpMatches[i];
            if(!pMP1 || !pMP2 || pMP1->isBad() || pMP2->isBad())
                continue;

            const int i2 = get<0>(pMP2->GetIndexInKeyFrame(pKF2));
            if(i2 < 0)
                continue;

            const Eigen::Vector3f Pc1 = Rcw * pMP1->GetWorldPos() + tcw;
            const float distance = Pc1.norm();
            if(!std::isfinite(distance) || distance <= 0.0f)
                continue;

            vDepthRefs.push_back({idx, i, distance});
        }
    }

    if(vDepthRefs.size() < static_cast<size_t>(minDepthBalancedMatches))
    {
        cout << "global final depth selection skipped: valid " << vDepthRefs.size() << endl;
        return vvpMatches;
    }

    std::sort(vDepthRefs.begin(), vDepthRefs.end(), [](const GlobalDepthMatchRef& a, const GlobalDepthMatchRef& b) {
        return a.depth < b.depth;
    });

    const size_t nTotal = vDepthRefs.size();
    const size_t nNearEnd = nTotal / 3;
    const size_t nFarBegin = (2 * nTotal) / 3;
    const float nearThreshold = vDepthRefs[nNearEnd - 1].depth;
    const float farThreshold = vDepthRefs[nFarBegin].depth;

    std::array<vector<GlobalDepthMatchRef>, 3> vDepthGroups;
    vDepthGroups[0].assign(vDepthRefs.begin(), vDepthRefs.begin() + nNearEnd);
    vDepthGroups[1].assign(vDepthRefs.begin() + nNearEnd, vDepthRefs.begin() + nFarBegin);
    vDepthGroups[2].assign(vDepthRefs.begin() + nFarBegin, vDepthRefs.end());

    const size_t nNear = vDepthGroups[0].size();
    const size_t nMiddle = vDepthGroups[1].size();
    const size_t nFar = vDepthGroups[2].size();
    const size_t nPerGroup = std::min(nNear, std::min(nMiddle, nFar));

    if(nPerGroup == 0 || 3 * nPerGroup < static_cast<size_t>(minDepthBalancedMatches))
    {
        cout << "global final depth selection skipped: near " << nNear
             << ", middle " << nMiddle
             << ", far " << nFar << endl;
        return vvpMatches;
    }

    std::array<size_t, 3> vOccupiedCells;
    vOccupiedCells.fill(0);
    std::array<size_t, 3> vSelectedPerDepth;
    vSelectedPerDepth.fill(0);

    for(size_t iGroup = 0; iGroup < vDepthGroups.size(); iGroup++)
    {
        vSelectedPerDepth[iGroup] = SelectGlobalGridDistributedRefs(vpKF1s, vDepthGroups[iGroup], nPerGroup,
                                                                    vvpMatches, vvpSelectedMatches,
                                                                    vOccupiedCells[iGroup]);
    }

    cout << "global final depth selection: valid " << nTotal
         << ", near " << nNear
         << ", middle " << nMiddle
         << ", far " << nFar
         << ", selected per group " << nPerGroup
         << ", selected " << CountMatches(vvpSelectedMatches) << endl;
    cout << "global 8x8 grid cells occupied: near " << vOccupiedCells[0]
         << ", middle " << vOccupiedCells[1]
         << ", far " << vOccupiedCells[2]
         << "; selected near " << vSelectedPerDepth[0]
         << ", middle " << vSelectedPerDepth[1]
         << ", far " << vSelectedPerDepth[2] << endl;
    cout << "global 3d distance stats: min " << PercentileDepth(vDepthRefs, 0.0f)
         << ", near threshold " << nearThreshold
         << ", median " << PercentileDepth(vDepthRefs, 0.50f)
         << ", far threshold " << farThreshold
         << ", max " << PercentileDepth(vDepthRefs, 1.0f) << endl;

    return vvpSelectedMatches;
}

void CalibC2C::RunCalib()
{
    vector<KeyFrame*> rsrc, rdst;
    vector<vector<MapPoint*>> mmps;

    int bestMatchesNum = 0;
    g2o::Sim3 g2oS12;

    cout << "front camera kf size: " << srcKFs.size() << endl;

    for(auto&& kf : srcKFs)
    {
        auto cand = DetectNBestCandidates(kf, bestCandNum);
        cout << "detect cand size: " << cand.size() << endl;

        KeyFrame* matchedKF;
        g2o::Sim3 gcw2;
        vector<MapPoint*> vpMPs;
        vector<MapPoint*> vpMatchedMPs;

        int matchesNum;
        bool valid = DetectCommonRegionsFromCand(kf, cand, matchedKF, gcw2, matchesNum, vpMPs, vpMatchedMPs);

        if(valid){
            if(matchesNum > bestMatchesNum){
                bestMatchesNum = matchesNum;
                g2o::Sim3 gcw1(kf->GetRotation().cast<double>(), kf->GetTranslation().cast<double>(), 1.0);
                // w1 -> w2
                g2oS12 = gcw2.inverse() * gcw1;

                // show best g2os12
                vector<float> euler = Converter::toEuler(Converter::toCvMat(g2oS12.rotation().toRotationMatrix()));
                cout << "---- before final optim ----" << endl;
                cout << "euler: ";
                for(auto&& e : euler)   cout << e * todeg << " ";
                cout << endl << "trans: ";
                Eigen::Vector3d t = g2oS12.translation();
                for(int i=0; i<3; i++)  cout << t(i) << " ";
                cout << endl;
            }
            rsrc.push_back(kf);
            rdst.push_back(matchedKF);
            mmps.push_back(vpMatchedMPs);
        }
    }

    if(rsrc.empty()){
        cout << "no common features detected!!" << endl;
        return;
    }

    cout << "matched kfs selected for global optim size: " << rsrc.size() << endl;
    cout << "matched mps selected during frame-to-frame matching size: " << CountMatches(mmps) << endl;
    printResult(g2oS12, "---- before first global optim ----");

    g2o::Sim3 firstPose = g2oS12;
    // global optimization at first place
    cout << "matched kfs involve in optim size: " << rsrc.size() << endl;
    int inliers = OptimizeSim3ForCalibr(rsrc, rdst, mmps, firstPose, 10, bFixScale);
    cout << "inliers size: " << inliers << endl;
    printResult(firstPose, "---- first pose optim ----");
    
    // global optimization at final place
    g2o::Sim3 finalPose = g2oS12;
    Eigen::Isometry3f finalPose1, finalPose2;
    // Twc camera expressed at world frame
    finalPose1.matrix() = srcKFs.back()->GetPoseInverse().matrix();
    finalPose2.matrix() = dstKFs.back()->GetPoseInverse().matrix();
    inliers = OptimizeSim3ForCalibr(rsrc, rdst, mmps, finalPose, 10, bFixScale, finalPose1, finalPose2);
    cout << "inliers size: " << inliers << endl;
    printResult(finalPose, "---- final pose optim ----");

}
