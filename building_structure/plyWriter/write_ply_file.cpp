#include <glog/logging.h>
#include <fstream> 
#include <string>
#include <vector>
#include <set>
#include <iterator>  
#include <stdio.h>
#include <iostream>
#include <cmath>

#include <theia/theia.h>

#include "dbscan.h"

using namespace theia;

DEFINE_string(reconstruction, "/TheiaSfM/iron/iron_mapping_hsl", "Theia Reconstruction file.");
DEFINE_string(ply_file, "/TheiaSfM/iron/plyWriter/iron_hsl.ply", "Output PLY file.");
DEFINE_int32(min_num_observations_per_point, 3,
             "Minimum number of observations for a point to be written out to "
             "the PLY file. This helps reduce noise in the resulty PLY file.");
DEFINE_string(filtering, "true", "Theia Reconstruction file.");

// Gather points from tracks.
void GatherTracks(const Reconstruction& reconstruction,
                  std::vector<Eigen::Vector3d>* points_to_write,
                  std::vector<Eigen::Vector3i>* colors_to_write) {
  for (const TrackId track_id : reconstruction.TrackIds()) {
    const Track& track = *reconstruction.Track(track_id);
    if(track.Color()[0] >= 150){ // used to be 150
      points_to_write->emplace_back(track.Point().hnormalized());
      colors_to_write->emplace_back(track.Color()[0],
                                track.Color()[1],
                                track.Color()[2]);
    }
  }
}

// Gather camera positions.
void GatherCameras(const Reconstruction& reconstruction,
                   std::vector<Eigen::Vector3d>* points_to_write,
                   std::vector<Eigen::Vector3i>* colors_to_write) {
  for (const ViewId view_id : reconstruction.ViewIds()) {
    const View& view = *reconstruction.View(view_id);
    if (!view.IsEstimated()) {
      continue;
    }
    points_to_write->emplace_back(view.Camera().GetPosition());
    colors_to_write->emplace_back(0, 255, 0);
  }

}


std::vector<Eigen::Vector3d> filterPoints(std::vector<Eigen::Vector3d> points_to_write){
  std::vector<Point> points;
  Point p;

  for(int i=0; i < points_to_write.size(); i++){
    p.x = points_to_write[i][0];
    p.y = points_to_write[i][1];
    p.z = points_to_write[i][2];
    p.clusterID = UNCLASSIFIED;
    points.push_back(p);
    // std::cout << p.x << p.y << p.z << std::endl;
  }
  
  DBSCAN ds(55, 0.03, points);
  ds.run();
  std::set<int> clusters;
  std::set<int>::iterator itr; 

  std::vector<int> clusters_content;

  std::vector<Eigen::Vector3d> points_new;

  for(int i=0; i < points.size(); i++){
    // std::cout<<ds.m_points[i].clusterID << std::endl;
    clusters.insert(ds.m_points[i].clusterID);
    clusters_content.push_back(ds.m_points[i].clusterID);
    if(ds.m_points[i].clusterID > 0){
      points_new.push_back(Eigen::Vector3d(ds.m_points[i].x, ds.m_points[i].y, ds.m_points[i].z));
    } 
  }
  std::cout << "DBSAN clusters:" << std::endl;

  for (itr = clusters.begin(); itr != clusters.end(); ++itr) 
  { 
      std::cout << '\t' << *itr << " Num: "<< std::count(clusters_content.begin(), clusters_content.end(), *itr ) << std::endl;

  } 

  // for(int i=0; i < clusters.size(); i++)
  //   std::cout << "Cluster" << clusters[i] << std::endl;
  return points_new;
}

void findEdges(const Reconstruction& reconstruction,
              std::vector<Eigen::Vector3i>* colors_to_write,
               std::vector<Eigen::Vector3d>* points_to_write,
               std::vector<double> &x, std::vector<double> &y, std::vector<double> &z){
  
  auto max = std::max_element(y.begin(), y.end());
  auto maxi = std::distance(y.begin(), max);

  auto min = std::min_element(y.begin(), y.end());
  auto mini = std::distance(y.begin(), min);


  (*colors_to_write)[maxi] = Eigen::Vector3i(0, 0, 255);
  (*colors_to_write)[mini] = Eigen::Vector3i(0, 0, 255);

  auto temp = ((*points_to_write)[maxi] -  (*points_to_write)[mini]);
  double dist = std::sqrt( double(temp.x()*temp.x() + temp.y()*temp.y()  + temp.z()*temp.z() ) );
  std::cout << "Distance between points " << dist << std::endl;
  
  double epsilon = 0.001;
  TrackId track_id_max, track_id_min;

  for (const TrackId track_id : reconstruction.TrackIds()) {
    const Track& track = *reconstruction.Track(track_id);
    if(std::fabs(track.Point().hnormalized()[0] - x[mini]) < epsilon &&
       std::fabs(track.Point().hnormalized()[1] - y[mini]) < epsilon &&
       std::fabs(track.Point().hnormalized()[2] - z[mini]) < epsilon){
      std::cout << "Found track for min point" << std::endl;
      track_id_min = track_id;
    } 
    if(std::fabs(track.Point().hnormalized()[0] - x[maxi]) < epsilon &&
       std::fabs(track.Point().hnormalized()[1] - y[maxi]) < epsilon &&
       std::fabs(track.Point().hnormalized()[2] - z[maxi]) < epsilon){
      std::cout << "Found track for min point" << std::endl;
      track_id_max = track_id;
    }  
  }
  bool found_min = false;
  bool found_max = false;

  for (const ViewId view_id : reconstruction.ViewIds()) {
    const View& view = *reconstruction.View(view_id);
    if (!view.IsEstimated()) {
      continue;
    }

    const Feature* feature_min = view.GetFeature(track_id_min);
    if(feature_min != NULL && !found_min){
        const int x = static_cast<int>((*feature_min).x());
        const int y = static_cast<int>((*feature_min).y());
        Eigen::Vector3d camera_position = view.Camera().GetPosition();
        found_min = true;
        std::cout << "Keypoint for a track min: " << x << " " << y << std::endl;
        std::cout << "Camera position for a point min: " << camera_position[0] << " "<< camera_position[1] << " "<< camera_position[2] <<std::endl;
    }


    const Feature* feature_max = view.GetFeature(track_id_max);
    if(feature_max != NULL && !found_max){
        const int x = static_cast<int>((*feature_max).x());
        const int y = static_cast<int>((*feature_max).y());
        Eigen::Vector3d camera_position = view.Camera().GetPosition();
        found_max = true;
        std::cout << "Keypoint for a track max: " << x << " " << y << std::endl;
        std::cout << "Camera position for a point max: " << camera_position[0] << " "<< camera_position[1] << " "<< camera_position[2] <<std::endl;
    }

    if(found_max && found_min){
      break;
    }
  }


}

void determineObjectSize(const Reconstruction& reconstruction,
                  std::vector<Eigen::Vector3d>* points_to_write,
                  std::vector<Eigen::Vector3i>* colors_to_write) {
  
  std::vector<double> x,y,z;
  for(int i=0; i < (*points_to_write).size(); i++){
    x.push_back( (*points_to_write)[i][0] );
    y.push_back( (*points_to_write)[i][1] );
    z.push_back( (*points_to_write)[i][2] );
  }

findEdges( reconstruction,
           colors_to_write,
           points_to_write,
           x, y, z);


}
// Writes a PLY file for viewing in software such as MeshLab.
bool writePlyFile(const std::string& ply_file,
                  const Reconstruction& const_reconstruction,
                  const int min_num_observations_per_point) {
  CHECK_GT(ply_file.length(), 0);

  // Return false if the file cannot be opened for writing.
  std::ofstream ply_writer(ply_file, std::ofstream::out);
  if (!ply_writer.is_open()) {
    LOG(ERROR) << "Could not open the file: " << ply_file
               << " for writing a PLY file.";
    return false;
  }

  // First, remove any points that are unestimated or do not have enough 3D
  // points.
  Reconstruction reconstruction = const_reconstruction;
  const auto& track_ids = reconstruction.TrackIds();
  for (const TrackId track_id : track_ids) {
    const Track& track = *reconstruction.Track(track_id);
    if (!track.IsEstimated() || track.NumViews() < min_num_observations_per_point) {
      reconstruction.RemoveTrack(track_id);
    }
  }

  // Extract points that we will write to the PLY file.
  std::vector<Eigen::Vector3d> points_to_write;
  std::vector<Eigen::Vector3i> colors_to_write;
  GatherTracks(reconstruction, &points_to_write, &colors_to_write);

  points_to_write = filterPoints(points_to_write);
  determineObjectSize(reconstruction, &points_to_write, &colors_to_write);
  
  GatherCameras(reconstruction, &points_to_write, &colors_to_write);

  

  ply_writer << "ply"
    << '\n' << "format ascii 1.0"
             << '\n' << "element vertex " << points_to_write.size()
    << '\n' << "property float x"
    << '\n' << "property float y"
    << '\n' << "property float z"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "end_header" << std::endl;

  for (int i = 0; i < points_to_write.size(); i++) {
    ply_writer << points_to_write[i].transpose() << " "
               << colors_to_write[i].transpose() << "\n";
  }

  return true;

}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  theia::Reconstruction reconstruction;
  CHECK(theia::ReadReconstruction(FLAGS_reconstruction, &reconstruction))
      << "Could not read Reconstruction files.";

  CHECK(writePlyFile(FLAGS_ply_file,
                     reconstruction,
                     FLAGS_min_num_observations_per_point))
      << "Could not write out PLY file.";
  return 0;
}