#include "PclUtils.h"
#include "DisparityMap.h"

const std::string PATH = "/home/kacper/Dokumenty/engineering_project/resources/";

int main() {
    // ---> reading disparity map from file, so far the map is not used in the next steps <---

    DisparityMap disparityMap = DisparityMap();
    disparityMap.readMapFromFile(PATH + "disp1.pfm");

    float* map = disparityMap.getDisparityMap();
    std::cout << "Size: " << disparityMap.getSize() << '\n';
    for (int i = 0; i < disparityMap.getSize(); ++i) {
        std::cout << map[i] << ' ';
    }

    std::cout << "\nEnd of writing map content (size = " << disparityMap.getSize() << ")\n";


    // ---> performing triangulation on a point cloud from file 'apple.pcd' <---

    auto cloud = createPointCloudFromInputFile(PATH + "apple.pcd");

    // for obtaining additional data about vertex
    std::vector<int> parts, states;

    auto triangles = performTriangulation(cloud, parts, states);

    //  maybe for future use
    pcl::io::saveVTKFile("triangles.vtk", triangles);

    drawObjectInVisualizer(triangles, "Results of performed triangulation", "triangles");

    return (0);
}
