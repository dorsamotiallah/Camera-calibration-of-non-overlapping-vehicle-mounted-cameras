#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "calib.hpp"

using namespace ORB_SLAM3;
using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace
{

void PrintUsage(const char* argv0)
{
    cerr << "Usage: " << argv0
         << " path_to_vocabulary camera_settings sequence_name output_csv" << endl
         << endl
         << "The camera settings must contain System.LoadAtlasFromFile. The atlas loaded is:" << endl
         << "  ./<System.LoadAtlasFromFile><sequence_name>.osa" << endl;
}

} // namespace

int main(int argc, char** argv)
{
    if(argc != 5)
    {
        PrintUsage(argv[0]);
        return 1;
    }

    const string vocabularyPath = argv[1];
    const string settingsPath = argv[2];
    const string sequenceName = argv[3];
    const string outputCsv = argv[4];

    SubSystem slam(vocabularyPath, settingsPath, System::MONOCULAR, false, 0, sequenceName);
    slam.clearSaveAtlasPath();
    Atlas* atlas = slam.getAtlas();
    if(!atlas)
    {
        cerr << "No atlas loaded." << endl;
        return 1;
    }

    std::ofstream out(outputCsv);
    if(!out.is_open())
    {
        cerr << "Could not open output CSV: " << outputCsv << endl;
        slam.Shutdown();
        return 1;
    }

    out << std::setprecision(9);
    out << "map_id,kf_id,mp_id,kf_time,cx,cy,cz,pw_x,pw_y,pw_z,kp_x,kp_y,img_min_x,img_min_y,img_max_x,img_max_y,octave,response\n";

    size_t nRows = 0;
    size_t nKFs = 0;
    size_t nMPs = 0;

    vector<Map*> maps = atlas->GetAllMaps();
    std::sort(maps.begin(), maps.end(), [](Map* a, Map* b) {
        if(!a || !b)
            return a < b;
        return a->GetId() < b->GetId();
    });

    for(Map* map : maps)
    {
        if(!map || map->IsBad())
            continue;

        vector<KeyFrame*> keyframes = map->GetAllKeyFrames();
        std::sort(keyframes.begin(), keyframes.end(), [](KeyFrame* a, KeyFrame* b) {
            if(!a || !b)
                return a < b;
            return a->mnId < b->mnId;
        });

        nKFs += keyframes.size();
        nMPs += map->GetAllMapPoints().size();

        for(KeyFrame* kf : keyframes)
        {
            if(!kf || kf->isBad())
                continue;

            const Eigen::Vector3f cameraCenter = kf->GetCameraCenter();
            const vector<MapPoint*> mapPoints = kf->GetMapPointMatches();
            const size_t n = std::min(mapPoints.size(), kf->mvKeysUn.size());

            for(size_t i = 0; i < n; ++i)
            {
                MapPoint* mp = mapPoints[i];
                if(!mp || mp->isBad())
                    continue;

                const Eigen::Vector3f worldPos = mp->GetWorldPos();
                const cv::KeyPoint& kp = kf->mvKeysUn[i];

                out << map->GetId() << ','
                    << kf->mnId << ','
                    << mp->mnId << ','
                    << kf->mTimeStamp << ','
                    << cameraCenter.x() << ','
                    << cameraCenter.y() << ','
                    << cameraCenter.z() << ','
                    << worldPos.x() << ','
                    << worldPos.y() << ','
                    << worldPos.z() << ','
                    << kp.pt.x << ','
                    << kp.pt.y << ','
                    << kf->mnMinX << ','
                    << kf->mnMinY << ','
                    << kf->mnMaxX << ','
                    << kf->mnMaxY << ','
                    << kp.octave << ','
                    << kp.response << '\n';
                ++nRows;
            }
        }
    }

    out.close();
    cout << "Exported " << nRows << " observations from " << nKFs
         << " keyframes and about " << nMPs << " map points to " << outputCsv << endl;

    slam.Shutdown();
    return 0;
}
