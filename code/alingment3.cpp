#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>






const float VOXEL_GRID_SIZE = 0.01;
const double NORMALS_RADIUS = 0.03;
const double FEATURES_RADIUS = 0.08;
const double SAC_MAX_CORRESPONDENCE_DIST = 1;
const double SAC_MIN_SAMPLE_DIST = 0.01;
const double FILTER_LIMIT = 1000;
const int MAX_SACIA_ITERATIONS = 500;





int test(int argc, char** argv)
{
    pcl::PointCloud<PointXYZRGB>::Ptr filterCloud( PointCloud<PointXYZRGB>::Ptr );
    pcl::PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud );
    pcl::PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloud<PointXYZRGB>::Ptr incloud, PointCloud<Normal>::Ptr normals );
    //void view( PointCloud<pcl::PointXYZRGB> & cloud );

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
            align( PointCloud<PointXYZRGB>::Ptr c1, PointCloud<PointXYZRGB>::Ptr c2,
                   pcl::PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud11 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud21 (new pcl::PointCloud<pcl::PointXYZRGB>);
    //open the clouds

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);


    pcl::io::loadPCDFile("/home/shiva/catkin_ws currentview2.pcd", *cloud11);
    pcl::io::loadPCDFile("/home/shiva/catkin_ws finalview2.pcd", *cloud21);


    //pass both voxel, through filters first

    cloud1= filterCloud( cloud11 );
    cloud2= filterCloud( cloud21 );

    //downsample the clouds, but store the downsampled clouds seperately
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1ds (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2ds (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<PointXYZRGB> vox_grid;
    vox_grid.setLeafSize( VOXEL_GRID_SIZE, VOXEL_GRID_SIZE, VOXEL_GRID_SIZE );
    vox_grid.setInputCloud( cloud1 );
    vox_grid.filter( *cloud1ds );

    vox_grid.setInputCloud( cloud2 );
    vox_grid.filter( *cloud2ds );


    //compute normals
    pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals( cloud1ds );
    pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals( cloud2ds );



    //compute local features
    pcl::PointCloud<FPFHSignature33>::Ptr features1 = getFeatures( cloud1ds, normals1 );
    pcl::PointCloud<FPFHSignature33>::Ptr features2 = getFeatures( cloud2ds, normals2 );




    //Get an initial estimate for the transformation using SAC
    //returns the transformation for cloud2 so that it is aligned with cloud1
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia = align( cloud1ds, cloud2ds, features1, features2 );
    Eigen::Matrix4f init_transform = sac_ia.getFinalTransformation();
    //transformPointCloud( *cloud2, *cloud2, init_transform );

    transformPointCloud( *cloud2ds, *cloud2ds, init_transform );

    pcl::PointCloud<pcl::PointXYZRGB> final=*cloud1;

    //ICP on the downsample clouds
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputCloud(cloud1ds);
    icp.setInputTarget(cloud2ds);
    icp.setMaximumIterations(100); //MAX_ICP_ITERATIONS is 100

    final.clear();
    icp.align(final); // returns the transformed source (input) as output. i.e. the transformation is for cloud1ds

    //align the full clouds based on the transformation
    Eigen::Matrix4f final_transform = icp.getFinalTransformation();
    transformPointCloud( *cloud1, *cloud1, final_transform );
    transformPointCloud( *cloud2, *cloud2, final_transform );

    //concatenate the clouds
    final.clear();
    final = *cloud1;

    final += *cloud2;
    view(final);
    return 1;
}

//computes the transformation for cloud2 so that it is transformed so that it is aligned with cloud1
pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
align( PointCloud<PointXYZRGB>::Ptr cloud1, PointCloud<PointXYZRGB>::Ptr cloud2,
       PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2 ) {

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
    Eigen::Matrix4f final_transformation;
    sac_ia.setInputCloud( cloud2 );
    sac_ia.setSourceFeatures( features2 );
    sac_ia.setInputTarget( cloud1 );
    sac_ia.setTargetFeatures( features1 );
    sac_ia.setMaximumIterations( MAX_SACIA_ITERATIONS );
    PointCloud<PointXYZRGB> finalcloud;
    sac_ia.align( finalcloud );
    return sac_ia;
}

pcl::PointCloud<FPFHSignature33>::Ptr getFeatures( PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<Normal>::Ptr normals ) {

    PointCloud<FPFHSignature33>::Ptr features = PointCloud<FPFHSignature33>::Ptr (new PointCloud<FPFHSignature33>);
    search::KdTree<PointXYZRGB>::Ptr search_method_ptr = search::KdTree<PointXYZRGB>::Ptr (new search::KdTree<PointXYZRGB>);
    FPFHEstimation<PointXYZRGB, Normal, FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud( cloud );
    fpfh_est.setInputNormals( normals );
    fpfh_est.setSearchMethod( search_method_ptr );
    fpfh_est.setRadiusSearch( FEATURES_RADIUS );
    fpfh_est.compute( *features );
    return features;
}

pcl::PointCloud<Normal>::Ptr getNormals( PointCloud<PointXYZRGB>::Ptr incloud ) {

    PointCloud<Normal>::Ptr normalsPtr = PointCloud<Normal>::Ptr (new PointCloud<Normal>);
    NormalEstimation<PointXYZRGB, Normal> norm_est;
    norm_est.setInputCloud( incloud );
    norm_est.setRadiusSearch( NORMALS_RADIUS );
    norm_est.compute( *normalsPtr );
    return normalsPtr;
}

pcl::PointCloud<PointXYZRGB>::Ptr downsample (PointCloud<PointXYZRGB>::Ptr pc ,float leafsize) {
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    sor.setInputCloud (pc);
    sor.setLeafSize (leafsize, leafsize, leafsize);
    sor.filter (*cloud_filtered);
    return cloud_filtered;

}

pcl::PointCloud<PointXYZRGB>::Ptr filterCloud (PointCloud<PointXYZRGB>::Ptr pc  ) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, FILTER_LIMIT);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, FILTER_LIMIT);
    std::cout<<"running";
    pass.filter(*cloud_filtered);

    return cloud_filtered;

}

//void view( PointCloud<pcl::PointXYZRGB> & cloud ) {

//        pcl::visualization::CloudViewer viewer1("Cloud Viewer");
//        viewer1.showCloud( cloud.makeShared() );
//    while( !viewer1.wasStopped() );

//        return;
//}
