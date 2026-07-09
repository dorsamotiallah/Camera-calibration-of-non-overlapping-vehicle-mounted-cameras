#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <Eigen/Core>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ORB_SLAM3;
using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;

namespace
{

class AtlasObservationExportSystem : public System
{
public:
    AtlasObservationExportSystem(const string &voc, const string &settings, const eSensor sensor, const string &sequence)
        : System(voc, settings, sensor, false, 0, sequence)
    {
    }

    Atlas* atlas() { return mpAtlas; }
};

void usage()
{
    cerr << "Usage:\n"
         << "  atlas_export_observations VOCAB SETTINGS SEQUENCE OUTPUT.csv\n\n"
         << "The settings file must contain System.LoadAtlasFromFile.\n"
         << "Writes raw keyframe keypoint-to-map-point observations from the loaded atlas,\n"
         << "including each keyframe's full pose (world-to-camera rotation as a quaternion,\n"
         << "plus the camera center already used to derive it).\n";
}

// qw,qx,qy,qz is kf->GetRotation() (Rcw, world-to-camera: p_cam = Rcw * p_world + tcw).
// camera_x,camera_y,camera_z is kf->GetCameraCenter() (Twc translation, camera position
// in world coordinates: p_world_camera_center = -Rcw^T * tcw). Together they give the
// full keyframe pose in either direction without needing tcw separately.
bool writeObservations(const string &path, const vector<Map*> &maps)
{
    std::ofstream csv(path);
    if(!csv.is_open())
        return false;

    csv << "map_id,kf_id,frame_id,timestamp,kp_idx,u,v,octave,angle,response,size,"
        << "mp_id,mp_x,mp_y,mp_z,camera_x,camera_y,camera_z,qw,qx,qy,qz\n";
    csv << std::fixed << std::setprecision(9);

    for(Map *map : maps)
    {
        if(!map || map->IsBad())
            continue;

        vector<KeyFrame*> keyframes = map->GetAllKeyFrames();
        std::sort(keyframes.begin(), keyframes.end(), KeyFrame::lId);

        for(KeyFrame *kf : keyframes)
        {
            if(!kf || kf->isBad())
                continue;

            const vector<MapPoint*> mapPoints = kf->GetMapPointMatches();
            const Eigen::Vector3f cameraCenter = kf->GetCameraCenter();
            const Eigen::Quaternionf qcw(kf->GetRotation());
            const size_t n = std::min(mapPoints.size(), kf->mvKeysUn.size());

            for(size_t idx = 0; idx < n; ++idx)
            {
                MapPoint *mp = mapPoints[idx];
                if(!mp || mp->isBad())
                    continue;

                const cv::KeyPoint &kp = kf->mvKeysUn[idx];
                const Eigen::Vector3f pos = mp->GetWorldPos();
                csv << map->GetId() << ","
                    << kf->mnId << ","
                    << kf->mnFrameId << ","
                    << kf->mTimeStamp << ","
                    << idx << ","
                    << kp.pt.x << ","
                    << kp.pt.y << ","
                    << kp.octave << ","
                    << kp.angle << ","
                    << kp.response << ","
                    << kp.size << ","
                    << mp->mnId << ","
                    << pos.x() << ","
                    << pos.y() << ","
                    << pos.z() << ","
                    << cameraCenter.x() << ","
                    << cameraCenter.y() << ","
                    << cameraCenter.z() << ","
                    << qcw.w() << ","
                    << qcw.x() << ","
                    << qcw.y() << ","
                    << qcw.z() << "\n";
            }
        }
    }

    return true;
}

} // namespace

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        usage();
        return 2;
    }

    const string vocab = argv[1];
    const string settings = argv[2];
    const string sequence = argv[3];
    const string output = argv[4];

    AtlasObservationExportSystem slam(vocab, settings, System::MONOCULAR, sequence);
    Atlas *atlas = slam.atlas();
    vector<Map*> maps = atlas->GetAllMaps();

    size_t totalKFs = 0;
    size_t totalMPs = 0;
    for(Map *map : maps)
    {
        if(!map || map->IsBad())
            continue;
        totalKFs += map->GetAllKeyFrames().size();
        totalMPs += map->GetAllMapPoints().size();
    }

    if(!writeObservations(output, maps))
    {
        cerr << "Could not write CSV: " << output << endl;
        return 1;
    }

    cout << "Loaded " << maps.size() << " maps, " << totalKFs
         << " keyframes, " << totalMPs << " raw map points." << endl;
    cout << "Wrote observations to " << output << endl;

    slam.Shutdown();
    return 0;
}
