#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Optimizer.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
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

class AtlasRefineSystem : public System
{
public:
    AtlasRefineSystem(const string &voc, const string &settings, const eSensor sensor, const string &sequence)
        : System(voc, settings, sensor, false, 0, sequence)
    {
    }

    Atlas* atlas() { return mpAtlas; }

    void setSaveAtlasPrefix(const string &prefix) { mStrSaveAtlasToFile = prefix; }
};

struct Options
{
    string vocab;
    string settings;
    string sequence;
    string outputPrefix;
    double scale = 1.0;
    int baIterations = 0;
    bool havePlane = false;
    Eigen::Vector3d planeNormal = Eigen::Vector3d::Zero();
    double planeD = 0.0;
    double cameraHeight = 0.0;
};

void usage()
{
    cerr << "Usage:\n"
         << "  atlas_metric_refine VOCAB SETTINGS SEQUENCE OUTPUT_PREFIX --scale S [options]\n\n"
         << "Options:\n"
         << "  --scale S                 Initial metric scale, meters per SLAM unit.\n"
         << "  --ba-iterations N         Optional standard ORB-SLAM global BA after scaling.\n"
         << "  --plane nx ny nz d        Optional ground plane in original atlas SLAM units.\n"
         << "  --camera-height H         Optional real camera height in meters for diagnostics.\n\n"
         << "The settings file must contain System.LoadAtlasFromFile. The output atlas is saved as:\n"
         << "  ./OUTPUT_PREFIX<SEQUENCE>.osa\n";
}

bool parseArgs(int argc, char **argv, Options &opts)
{
    if(argc < 6)
        return false;

    opts.vocab = argv[1];
    opts.settings = argv[2];
    opts.sequence = argv[3];
    opts.outputPrefix = argv[4];

    for(int i = 5; i < argc; )
    {
        const string arg = argv[i];
        if(arg == "--scale" && i + 1 < argc)
        {
            opts.scale = std::stod(argv[i + 1]);
            i += 2;
        }
        else if(arg == "--ba-iterations" && i + 1 < argc)
        {
            opts.baIterations = std::stoi(argv[i + 1]);
            i += 2;
        }
        else if(arg == "--plane" && i + 4 < argc)
        {
            opts.planeNormal[0] = std::stod(argv[i + 1]);
            opts.planeNormal[1] = std::stod(argv[i + 2]);
            opts.planeNormal[2] = std::stod(argv[i + 3]);
            opts.planeD = std::stod(argv[i + 4]);
            opts.havePlane = true;
            i += 5;
        }
        else if(arg == "--camera-height" && i + 1 < argc)
        {
            opts.cameraHeight = std::stod(argv[i + 1]);
            i += 2;
        }
        else
        {
            cerr << "Unknown or incomplete option: " << arg << endl;
            return false;
        }
    }

    if(!std::isfinite(opts.scale) || opts.scale <= 0.0)
    {
        cerr << "--scale must be positive." << endl;
        return false;
    }

    if(opts.havePlane && opts.planeNormal.norm() <= 1e-12)
    {
        cerr << "--plane normal must be nonzero." << endl;
        return false;
    }

    return true;
}

Map* chooseLargestMap(Atlas *atlas)
{
    vector<Map*> maps = atlas->GetAllMaps();
    Map *best = nullptr;
    size_t bestKFs = 0;

    for(Map *map : maps)
    {
        if(!map)
            continue;
        const size_t nKFs = map->GetAllKeyFrames().size();
        if(!best || nKFs > bestKFs)
        {
            best = map;
            bestKFs = nKFs;
        }
    }

    return best;
}

double median(vector<double> values)
{
    if(values.empty())
        return std::numeric_limits<double>::quiet_NaN();

    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    double med = values[mid];
    if(values.size() % 2 == 0)
    {
        std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
        med = 0.5 * (med + values[mid - 1]);
    }
    return med;
}

void reportHeightStats(Map *map, Eigen::Vector3d normal, double d, const string &label, double cameraHeight)
{
    const double norm = normal.norm();
    if(norm <= 1e-12)
        return;

    normal /= norm;
    d /= norm;

    vector<double> heights;
    for(KeyFrame *kf : map->GetAllKeyFrames())
    {
        if(!kf || kf->isBad())
            continue;
        const Eigen::Vector3d center = kf->GetCameraCenter().cast<double>();
        heights.push_back(std::abs(normal.dot(center) + d));
    }

    if(heights.empty())
        return;

    const double med = median(heights);
    cout << label << " median camera-plane height: " << med;
    if(cameraHeight > 0.0)
        cout << " (real height " << cameraHeight << ", error " << med - cameraHeight << ")";
    cout << endl;
}

void scaleMap(Map *map, double scale)
{
    for(KeyFrame *kf : map->GetAllKeyFrames())
    {
        if(!kf || kf->isBad())
            continue;

        Sophus::SE3f Twc = kf->GetPoseInverse();
        Twc.translation() *= static_cast<float>(scale);
        kf->SetPose(Twc.inverse());
    }

    for(MapPoint *mp : map->GetAllMapPoints())
    {
        if(!mp || mp->isBad())
            continue;

        mp->SetWorldPos(mp->GetWorldPos() * static_cast<float>(scale));
        mp->UpdateNormalAndDepth();
    }
}

} // namespace

int main(int argc, char **argv)
{
    Options opts;
    if(!parseArgs(argc, argv, opts))
    {
        usage();
        return 2;
    }

    AtlasRefineSystem slam(opts.vocab, opts.settings, System::MONOCULAR, opts.sequence);
    Atlas *atlas = slam.atlas();
    Map *map = chooseLargestMap(atlas);
    if(!map || map->GetAllKeyFrames().empty())
    {
        cerr << "No non-empty map found in loaded atlas." << endl;
        return 1;
    }
    atlas->ChangeMap(map);

    cout << "Loaded atlas map with " << map->GetAllKeyFrames().size()
         << " keyframes and " << map->GetAllMapPoints().size() << " map points." << endl;

    if(opts.havePlane)
        reportHeightStats(map, opts.planeNormal, opts.planeD, "Before scaling", opts.cameraHeight / opts.scale);

    scaleMap(map, opts.scale);

    if(opts.havePlane)
        reportHeightStats(map, opts.planeNormal, opts.planeD * opts.scale, "After scaling", opts.cameraHeight);

    if(opts.baIterations > 0)
    {
        cout << "Running standard ORB-SLAM global BA for " << opts.baIterations << " iterations..." << endl;
        Optimizer::GlobalBundleAdjustemnt(map, opts.baIterations);
        if(opts.havePlane)
            reportHeightStats(map, opts.planeNormal, opts.planeD * opts.scale, "After BA", opts.cameraHeight);
    }

    slam.setSaveAtlasPrefix(opts.outputPrefix);
    slam.Shutdown();

    cout << "Saved refined atlas prefix: " << opts.outputPrefix << endl;
    return 0;
}
