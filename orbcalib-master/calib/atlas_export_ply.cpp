#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include <Eigen/Core>
#include <algorithm>
#include <fstream>
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

class AtlasExportSystem : public System
{
public:
    AtlasExportSystem(const string &voc, const string &settings, const eSensor sensor, const string &sequence)
        : System(voc, settings, sensor, false, 0, sequence)
    {
    }

    Atlas* atlas() { return mpAtlas; }
};

struct Vertex
{
    Eigen::Vector3f p;
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

void usage()
{
    cerr << "Usage:\n"
         << "  atlas_export_ply VOCAB SETTINGS SEQUENCE OUTPUT.ply [--keyframes-only]\n\n"
         << "The settings file must contain System.LoadAtlasFromFile.\n"
         << "Map points are written in light gray; keyframe camera centers are blue.\n";
}

void addMapPoints(Map *map, vector<Vertex> &vertices)
{
    for(MapPoint *mp : map->GetAllMapPoints())
    {
        if(!mp || mp->isBad())
            continue;

        vertices.push_back({mp->GetWorldPos(), 210, 210, 210});
    }
}

void addKeyFrames(Map *map, vector<Vertex> &vertices)
{
    vector<KeyFrame*> keyframes = map->GetAllKeyFrames();
    std::sort(keyframes.begin(), keyframes.end(), KeyFrame::lId);

    for(KeyFrame *kf : keyframes)
    {
        if(!kf || kf->isBad())
            continue;

        vertices.push_back({kf->GetCameraCenter(), 35, 105, 255});
    }
}

bool writePly(const string &path, const vector<Vertex> &vertices)
{
    std::ofstream out(path);
    if(!out.is_open())
        return false;

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vertices.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";

    for(const Vertex &v : vertices)
    {
        out << v.p.x() << " " << v.p.y() << " " << v.p.z() << " "
            << static_cast<int>(v.r) << " "
            << static_cast<int>(v.g) << " "
            << static_cast<int>(v.b) << "\n";
    }

    return true;
}

} // namespace

int main(int argc, char **argv)
{
    if(argc < 5 || argc > 6)
    {
        usage();
        return 2;
    }

    const string vocab = argv[1];
    const string settings = argv[2];
    const string sequence = argv[3];
    const string output = argv[4];
    const bool keyframesOnly = argc == 6 && string(argv[5]) == "--keyframes-only";

    if(argc == 6 && !keyframesOnly)
    {
        usage();
        return 2;
    }

    AtlasExportSystem slam(vocab, settings, System::MONOCULAR, sequence);
    Atlas *atlas = slam.atlas();
    vector<Map*> maps = atlas->GetAllMaps();

    vector<Vertex> vertices;
    size_t totalKFs = 0;
    size_t totalMPs = 0;
    for(Map *map : maps)
    {
        if(!map)
            continue;

        totalKFs += map->GetAllKeyFrames().size();
        totalMPs += map->GetAllMapPoints().size();
        if(!keyframesOnly)
            addMapPoints(map, vertices);
        addKeyFrames(map, vertices);
    }

    if(vertices.empty())
    {
        cerr << "No vertices to export from loaded atlas." << endl;
        return 1;
    }

    if(!writePly(output, vertices))
    {
        cerr << "Could not write PLY: " << output << endl;
        return 1;
    }

    cout << "Loaded " << maps.size() << " maps, " << totalKFs
         << " keyframes, " << totalMPs << " raw map points." << endl;
    cout << "Wrote " << vertices.size() << " vertices to " << output << endl;

    slam.Shutdown();
    return 0;
}
