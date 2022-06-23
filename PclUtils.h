#ifndef PclUtils_H
#define PclUtils_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>

const int K = 20; // parameter - number of nearest neighbours for estimation (so far const but this may change)


/* ---> universal functions <--- */

pcl::PCLPointCloud2 loadPcdFile(const std::string& filename);

/* ---> methods for performing triangulation  <--- */

pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudFromInputFile(const std::string& filename);

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

pcl::PointCloud<pcl::PointNormal>::Ptr createCloudWithNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                              const pcl::PointCloud<pcl::Normal>::Ptr& normals);

pcl::GreedyProjectionTriangulation<pcl::PointNormal> createGptObject(double radius, double mu, int maxNearestNeighbors,
                                                                     double epsAngle, double minAngle, double maxAngle);

pcl::PolygonMesh performTriangulation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& parts,
                                      std::vector<int>& states);

void drawObjectInVisualizer(const pcl::PolygonMesh& mesh, const std::string& title, const std::string& meshId);


#endif //PclUtils_H