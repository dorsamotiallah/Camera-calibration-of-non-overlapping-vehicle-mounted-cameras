#include "Sim3Solver.h"
#include "calib.hpp"
#include "edge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>

using std::cout;
using std::endl;
using namespace ORB_SLAM3;

const float todeg = 180 / M_PI;

static string KeyframeMatchDebugCsvPath()
{
    const char* path = std::getenv("CALIB_KEYFRAME_MATCHES_CSV");
    if(path && path[0] != '\0')
        return string(path);
    return "calib_keyframe_matches.csv";
}

static string CsvEscape(const string& value)
{
    string escaped = "\"";
    for(char c : value)
    {
        if(c == '"')
            escaped += "\"\"";
        else
            escaped += c;
    }
    escaped += "\"";
    return escaped;
}

static void InitKeyframeMatchDebugCsv(const string& path)
{
    std::ofstream csv(path);
    if(!csv.is_open())
    {
        cout << "WARNING: could not write keyframe match debug CSV: " << path << endl;
        return;
    }

    csv << "stage,src_kf_id,dst_kf_id,src_frame_id,dst_frame_id,src_timestamp,dst_timestamp,src_frame,dst_frame,"
        << "src_kp_idx,dst_kp_idx,src_u,src_v,dst_u,dst_v,"
        << "src_mp_id,dst_mp_id,src_mp_x,src_mp_y,src_mp_z,dst_mp_x,dst_mp_y,dst_mp_z\n";
}

static bool GetMapPointObservation(MapPoint* mp, KeyFrame* preferredKF, KeyFrame*& observedKF, int& idx)
{
    observedKF = nullptr;
    idx = -1;
    if(!mp || mp->isBad())
        return false;

    if(preferredKF && !preferredKF->isBad())
    {
        const auto preferredIdx = mp->GetIndexInKeyFrame(preferredKF);
        const int candidateIdx = get<0>(preferredIdx);
        if(candidateIdx >= 0 && candidateIdx < preferredKF->mvKeysUn.size())
        {
            observedKF = preferredKF;
            idx = candidateIdx;
            return true;
        }
    }

    const auto observations = mp->GetObservations();
    for(const auto& observation : observations)
    {
        KeyFrame* kf = observation.first;
        const int candidateIdx = get<0>(observation.second);
        if(kf && !kf->isBad() && candidateIdx >= 0 && candidateIdx < kf->mvKeysUn.size())
        {
            observedKF = kf;
            idx = candidateIdx;
            return true;
        }
    }

    return false;
}

static void AppendKeyframeMatchDebugCsv(const string& path, const string& stage, KeyFrame* srcKF,
    KeyFrame* fallbackDstKF, const vector<MapPoint*>& matchedMPs, const vector<KeyFrame*>* matchedKFs = nullptr)
{
    if(!srcKF || srcKF->isBad())
        return;

    const vector<MapPoint*>& srcMPs = srcKF->GetMapPointMatches();
    const size_t n = std::min(srcMPs.size(), matchedMPs.size());
    std::ofstream csv(path, std::ios::app);
    if(!csv.is_open())
    {
        cout << "WARNING: could not append keyframe match debug CSV: " << path << endl;
        return;
    }
    csv << std::fixed << std::setprecision(9);

    for(size_t i = 0; i < n; ++i)
    {
        MapPoint* srcMP = srcMPs[i];
        MapPoint* dstMP = matchedMPs[i];
        if(!srcMP || !dstMP || srcMP->isBad() || dstMP->isBad() || i >= srcKF->mvKeysUn.size())
            continue;

        KeyFrame* preferredDstKF = fallbackDstKF;
        if(matchedKFs && i < matchedKFs->size() && (*matchedKFs)[i])
            preferredDstKF = (*matchedKFs)[i];

        KeyFrame* dstKF = nullptr;
        int dstIdx = -1;
        if(!GetMapPointObservation(dstMP, preferredDstKF, dstKF, dstIdx))
            continue;

        const cv::KeyPoint& srcKP = srcKF->mvKeysUn[i];
        const cv::KeyPoint& dstKP = dstKF->mvKeysUn[dstIdx];
        const Eigen::Vector3f srcPos = srcMP->GetWorldPos();
        const Eigen::Vector3f dstPos = dstMP->GetWorldPos();

        csv << stage << ","
            << srcKF->mnId << "," << dstKF->mnId << ","
            << srcKF->mnFrameId << "," << dstKF->mnFrameId << ","
            << srcKF->mTimeStamp << "," << dstKF->mTimeStamp << ","
            << CsvEscape(srcKF->mNameFile) << "," << CsvEscape(dstKF->mNameFile) << ","
            << i << "," << dstIdx << ","
            << srcKP.pt.x << "," << srcKP.pt.y << ","
            << dstKP.pt.x << "," << dstKP.pt.y << ","
            << srcMP->mnId << "," << dstMP->mnId << ","
            << srcPos(0) << "," << srcPos(1) << "," << srcPos(2) << ","
            << dstPos(0) << "," << dstPos(1) << "," << dstPos(2) << "\n";
    }
}

CalibC2C::CalibC2C(System* src, System* dst, const MapScaleConfig& scaleConfig)
    : mScaleConfig(scaleConfig)
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

    if(mScaleConfig.useGlobalMapScales)
    {
        cout << "global map scale calibration enabled" << endl;
        cout << "  camera1 global scale: " << mScaleConfig.camera1GlobalScale << " m / SLAM unit" << endl;
        cout << "  camera2 global scale: " << mScaleConfig.camera2GlobalScale << " m / SLAM unit" << endl;
        cout << "  fix Sim3 scale after global scaling: " << (mScaleConfig.fixScaleAfterGlobalScaling ? "yes" : "no") << endl;
    }
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
        AppendKeyframeMatchDebugCsv(KeyframeMatchDebugCsvPath(), "bow", pKF, pMostBoWMatchesKF, vpMatchedPoints, &vpKeyFrameMatchedMP);

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
                AppendKeyframeMatchDebugCsv(KeyframeMatchDebugCsvPath(), "projection_initial", pKF, pMostBoWMatchesKF, vpMatchedMPs, &vpKeyFrameMatchedMP);

                // 5. after use searchByProjection to get more mp, optimize Scm with them
                if(numProjMatches >= 25)
                {
                    Eigen::Matrix<double, 7, 7> mHessian7x7;
                    int numOptMatches = Optimizer::OptimizeSim3(pKF, pKFi, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);
                    cout << "optim sim3 get size: " << numOptMatches << endl;
                    AppendKeyframeMatchDebugCsv(KeyframeMatchDebugCsvPath(), "projection_after_optimize_sim3", pKF, pMostBoWMatchesKF, vpMatchedMPs, &vpKeyFrameMatchedMP);

                    // 6. when optimed Scm is found, search by projection again to get more matched points 
                    if(numOptMatches >= 10)
                    {
                        gScw = gScm * gSmw;
                        vpMatchedMPs.assign(nMPNums, static_cast<MapPoint*>(nullptr));
                        // shrink search range
                        numProjMatches = matcher->SearchByProjection(pKF, mScw, vpMapPoints, vpMatchedMPs, 5, 2.0);
                        cout << "after optim sim3 search by projection get size: " << numProjMatches << endl;
                        AppendKeyframeMatchDebugCsv(KeyframeMatchDebugCsvPath(), "projection_after_optimized_pose", pKF, pMostBoWMatchesKF, vpMatchedMPs);

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

                            // 8. save best result along the iteration
                            if(nBestMatchesReproj < numProjMatches)
                            {
                                nBestMatchesReproj = numProjMatches; 
                                nBestNumCoindicendes = validKF;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMPs;
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
    g2o::Sim3 &g2oS12, const float th2, const bool bFixScale, Eigen::Isometry3f finalPose1, Eigen::Isometry3f finalPose2,
    const string& debugInliersCsvPath)
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
    const bool effectiveFixScale =
        bFixScale ||
        (mScaleConfig.useGlobalMapScales && mScaleConfig.fixScaleAfterGlobalScaling);
    g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale = effectiveFixScale;
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
    struct DebugRecord
    {
        unsigned long kf1Id;
        unsigned long kf2Id;
        unsigned long mp1Id;
        unsigned long mp2Id;
        Eigen::Vector3d p1;
        Eigen::Vector3d p2;
    };
    vector<vector<DebugRecord>> vvpDebugRecords(nKF);

    for(int idx=0; idx<nKF; idx++)
    {
        KeyFrame *pKF1 = vpKF1s[idx];
        KeyFrame *pKF2 = vpKF2s[idx];
        const vector<MapPoint*>& vpMapPoints = pKF1->GetMapPointMatches();
        const vector<MapPoint*>& vpMatches = vvpMatches[idx];

        int N = vpMatches.size();
        vector<g2o::EdgeSim3ProjectXYZForCalibr*> vpEdges12;       
        vector<g2o::EdgeInverseSim3ProjectXYZForCalibr*> vpEdges21;
        vector<DebugRecord> vpDebugRecords;
        vpEdges12.reserve(N);
        vpEdges21.reserve(N);
        vpDebugRecords.reserve(N);

        Eigen::Matrix3d R1w, R2w;
        Eigen::Vector3d t1w, t2w;
        Eigen::Isometry3f P1w, P2w;
        P1w.matrix() = pKF1->GetPose().matrix() * finalPose1.matrix();
        P2w.matrix() = pKF2->GetPose().matrix() * finalPose2.matrix();
        R1w = P1w.rotation().cast<double>();
        R2w = P2w.rotation().cast<double>();
        t1w = P1w.translation().cast<double>();
        t2w = P2w.translation().cast<double>();

        const double scale1 = mScaleConfig.useGlobalMapScales ? mScaleConfig.camera1GlobalScale : 1.0;
        const double scale2 = mScaleConfig.useGlobalMapScales ? mScaleConfig.camera2GlobalScale : 1.0;
        if(mScaleConfig.useGlobalMapScales)
        {
            t1w *= scale1;
            t2w *= scale2;
        }

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
            if(mScaleConfig.useGlobalMapScales)
                P3D1w *= static_cast<float>(scale1);
            vPoint1->setEstimate(P3D1w.cast<double>());
            vPoint1->setId(id1);
            vPoint1->setFixed(true);
            optimizer.addVertex(vPoint1);

            // mp see from camera 2
            g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
            Eigen::Vector3f P3D2w = pMP2->GetWorldPos();
            P3D2w = finalPose2.inverse() * P3D2w;
            if(mScaleConfig.useGlobalMapScales)
                P3D2w *= static_cast<float>(scale2);
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
            vpDebugRecords.push_back({
                pKF1->mnId,
                pKF2->mnId,
                pMP1->mnId,
                pMP2->mnId,
                P3D1w.cast<double>(),
                P3D2w.cast<double>()});
        }

        vvpEdges12[idx] = vpEdges12;
        vvpEdges21[idx] = vpEdges21;
        vvpDebugRecords[idx] = vpDebugRecords;
    }

    if(mScaleConfig.useGlobalMapScales)
    {
        cout << "global map scales applied: correspondences=" << nCorrespondences << endl;
    }

    if(nCorrespondences == 0)
    {
        cout << "WARNING: no correspondences were added to calibration optimizer" << endl;
        return 0;
    }

    // 4. start optimize
    int outliers;
    for(int i=0; i<4; i++)
    {
        int activeEdgePairs = 0;
        for(int j=0; j<nKF; j++)
        {
            auto& vpEdges12 = vvpEdges12[j];
            auto& vpEdges21 = vvpEdges21[j];
            for(int k=0; k<vpEdges12.size(); k++)
            {
                auto e12 = vpEdges12[k];
                auto e21 = vpEdges21[k];
                if(e12 && e21 && e12->level() == 0 && e21->level() == 0)
                    activeEdgePairs++;
            }
        }

        if(activeEdgePairs == 0)
        {
            cout << "WARNING: calibration optimizer has no active edge pairs after outlier rejection" << endl;
            return 0;
        }

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

    if(nCorrespondences - outliers <= 0)
    {
        cout << "WARNING: calibration optimizer rejected all correspondences" << endl;
        return 0;
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

    if(!debugInliersCsvPath.empty())
    {
        std::ofstream csv(debugInliersCsvPath);
        if(csv.is_open())
        {
            csv << "kf1_id,kf2_id,mp1_id,mp2_id,"
                << "p1_x,p1_y,p1_z,p2_x,p2_y,p2_z,"
                << "p1_in_w2_x,p1_in_w2_y,p1_in_w2_z,"
                << "p2_in_w1_x,p2_in_w1_y,p2_in_w1_z,"
                << "chi2_12,chi2_21\n";

            const g2o::Sim3 g2oS21 = g2oS12.inverse();
            int written = 0;
            for(int j=0; j<nKF; j++)
            {
                auto& vpEdges12 = vvpEdges12[j];
                auto& vpEdges21 = vvpEdges21[j];
                auto& vpDebugRecords = vvpDebugRecords[j];

                for(int k=0; k<vpEdges12.size(); k++)
                {
                    auto e12 = vpEdges12[k];
                    auto e21 = vpEdges21[k];
                    if(!e12 || !e21 || e12->chi2() > th2 || e21->chi2() > th2)
                        continue;

                    const DebugRecord& rec = vpDebugRecords[k];
                    const Eigen::Vector3d p1InW2 = g2oS12.map(rec.p1);
                    const Eigen::Vector3d p2InW1 = g2oS21.map(rec.p2);

                    csv << rec.kf1Id << "," << rec.kf2Id << ","
                        << rec.mp1Id << "," << rec.mp2Id << ","
                        << rec.p1.x() << "," << rec.p1.y() << "," << rec.p1.z() << ","
                        << rec.p2.x() << "," << rec.p2.y() << "," << rec.p2.z() << ","
                        << p1InW2.x() << "," << p1InW2.y() << "," << p1InW2.z() << ","
                        << p2InW1.x() << "," << p2InW1.y() << "," << p2InW1.z() << ","
                        << e12->chi2() << "," << e21->chi2() << "\n";
                    written++;
                }
            }
            cout << "wrote calibration inliers: " << debugInliersCsvPath
                 << " rows=" << written << endl;
        }
        else
        {
            cout << "WARNING: could not write calibration inlier CSV: "
                 << debugInliersCsvPath << endl;
        }
    }

    return nCorrespondences - outliers;
}

void printResult(const g2o::Sim3& pose, const string& header)
{
    vector<float> euler = Converter::toEuler(Converter::toCvMat(pose.rotation().toRotationMatrix()));
    cout << header << endl;
    cout << "euler: ";
    for(auto&& e : euler)   cout << e * todeg << " ";
    cout << endl << "scale: " << pose.scale();
    cout << endl << "trans: ";
    Eigen::Vector3d t = pose.translation();
    for(int i=0; i<3; i++)  cout << t(i) << " ";
    cout << endl;
}

g2o::Sim3 ConvertRawSim3ToScaledMaps(const g2o::Sim3& rawS12, const MapScaleConfig& scaleConfig)
{
    if(!scaleConfig.useGlobalMapScales)
        return rawS12;

    const double scale1 = scaleConfig.camera1GlobalScale;
    const double scale2 = scaleConfig.camera2GlobalScale;

    // raw:    X2_raw = s * R * X1_raw + t
    // scaled: X1_m = scale1 * X1_raw, X2_m = scale2 * X2_raw
    // hence:  X2_m = (scale2 / scale1) * s * R * X1_m + scale2 * t
    //
    // If the caller asks to fix scale after applying metric map scales, use an
    // SE3 initial estimate. The height-derived scales are then the metric
    // authority and the camera-to-camera calibration should not carry a
    // leftover Sim3 scale from the raw monocular maps.
    const double scaledSim3Scale = scaleConfig.fixScaleAfterGlobalScaling
        ? 1.0
        : rawS12.scale() * scale2 / scale1;
    return g2o::Sim3(
        rawS12.rotation(),
        rawS12.translation() * scale2,
        scaledSim3Scale);
}

void CalibC2C::RunCalib()
{
    vector<KeyFrame*> rsrc, rdst;
    vector<vector<MapPoint*>> mmps;

    int bestMatchesNum = 0;
    g2o::Sim3 g2oS12;

    const string keyframeMatchDebugCsv = KeyframeMatchDebugCsvPath();
    InitKeyframeMatchDebugCsv(keyframeMatchDebugCsv);
    cout << "keyframe match debug CSV: " << keyframeMatchDebugCsv << endl;
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
                cout << "---- before final optim raw atlas units ----" << endl;
                cout << "euler: ";
                for(auto&& e : euler)   cout << e * todeg << " ";
                cout << endl << "scale: " << g2oS12.scale();
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
    
    g2o::Sim3 firstPose = ConvertRawSim3ToScaledMaps(g2oS12, mScaleConfig);
    if(mScaleConfig.useGlobalMapScales)
        printResult(firstPose, "---- scaled-map initial pose ----");

    // global optimization at first place
    cout << "matched kfs involve in optim size: " << rsrc.size() << endl;
    int inliers = OptimizeSim3ForCalibr(rsrc, rdst, mmps, firstPose, 10, bFixScale,
        Eigen::Isometry3f::Identity(), Eigen::Isometry3f::Identity(), "calib_inliers_first.csv");
    cout << "inliers size: " << inliers << endl;
    printResult(firstPose, "---- first pose optim ----");
    
    // global optimization at final place
    g2o::Sim3 finalPose = ConvertRawSim3ToScaledMaps(g2oS12, mScaleConfig);
    Eigen::Isometry3f finalPose1, finalPose2;
    // Twc camera expressed at world frame
    finalPose1.matrix() = srcKFs.back()->GetPoseInverse().matrix();
    finalPose2.matrix() = dstKFs.back()->GetPoseInverse().matrix();
    inliers = OptimizeSim3ForCalibr(rsrc, rdst, mmps, finalPose, 10, bFixScale, finalPose1, finalPose2,
        "calib_inliers_final.csv");
    cout << "inliers size: " << inliers << endl;
    printResult(finalPose, "---- final pose optim ----");

}
