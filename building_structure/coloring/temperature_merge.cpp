#include <Eigen/Core>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <theia/theia.h>

#include <string>
#include "color_conversion.h"

using namespace theia;

DEFINE_string(image_directory, "/TheiaSfM/iron/iron/rgb/",
              "Full path to the directory containing the images used to create "
              "the reconstructions. Must contain a trailing slash.");
DEFINE_string(input_reconstruction_file, "/TheiaSfM/iron/reconstruction-0", "Input reconstruction file.");
DEFINE_string(output_reconstruction_file, "/TheiaSfM/iron/iron_mapping_hsl", "Output reconstruction file.");
DEFINE_int32(num_threads, 1, "Number of threads to use for multithreading.");
DEFINE_int32(recontruction_type, 1, "1-thermal, 2-combined");




void ExtractColorsFromImage(
    const std::string& image_file,
    const std::string& image_file_thermal,
    const std::string& image_filepath_combined,
    const View& view,
    std::unordered_map<TrackId, Eigen::Vector3f>* colors,
    std::mutex* mutex_lock) {
  LOG(INFO) << "Extracting color for features in image: " << image_file;
  FloatImage image(image_file);
  FloatImage imageT(image_file_thermal);
  FloatImage combinedIm(image_filepath_combined);
  FloatImage imageThGr = imageT.AsGrayscaleImage();
  // imageT.Resize(image.Rows(), image.Cols());
  // std::cout << imageT.Channels() << std::endl;
  const auto& track_ids = view.TrackIds();
  if (image.Channels() == 3) {
    for (const TrackId track_id : track_ids) {
      const Feature feature = *view.GetFeature(track_id);
      const int x = static_cast<int>(feature.x());
      const int y = static_cast<int>(feature.y());
      std::lock_guard<std::mutex> lock(*mutex_lock);
      //std::cout << imageT.GetXY(x, y) << std::endl;
      // std::cout << imageT.GetXY(x, y) << " values" <<std::endl;
      // Eigen::Vector3f temperature(float(255.0), float(255), float(255));

/*      if (x < 160 && y < 120){
        value = imageT.GetXY(x, y);
        temperature = {255.0*value, 0, 255.0*value};
        std::cout << temperature << std::endl;
      }*/
      //std::cout << (*colors)[track_id] << std::endl;
      // 255.0 * image.GetXY(x, y) + 
      // std::cout << 255*imageT.GetXY(x, y)<<"Term"<<std::endl;
      // std::cout << 255*image.GetXY(x, y)<<"Rgb"<<std::endl;
      // (*colors)[track_id] += 255.0 * imageT.GetXY(x, y);
      // std::cout<<imageThGr.GetXY(x,y)<<"temp"<<std::endl;
      Eigen::Vector3f temperature(255*float(imageThGr.GetXY(x,y)[0]), 0, 255-255*float(imageThGr.GetXY(x,y)[0]));


      // rgb rgbT = {
      // 	temperature[0]/double(255),
      // 	0,
      // 	0.01,
      // };
      // //   rgb rgbT = {
      // // 	temperature[0]/double(255),
      // // 	temperature[1]/double(255),
      // // 	temperature[2]/double(255),
      // // };
      // rgb rgbI = {
      // 	color_image[0],
      // 	color_image[1],
      // 	color_image[2],      	
      // };
      // hsl hslT = rgb2hsl(rgbT);
      // hsl hslR = rgb2hsl(rgbI);

      // hsl fusion = {
      // 	hslT.h,
      // 	0,
      // 	0,
      // };

      // rgb fusionR = hsl2rgb(fusion);
      // // rgb fusionR = fusion;

      // // Eigen::Vector3f fusion_thermal_rgb(fusionR.r * 255, fusionR.g * 255, fusionR.b * 255);
      // Eigen::Vector3f fusion_thermal_rgb(fusion.h * 255, fusion.s * 255, fusion.l * 255);
      // // my mapping generisan ply21 koji samo pojacava crveni kanal
      // Eigen::Vector3f mymapping;
      // if(temperature[0] > 200){
      // 	mymapping = Eigen::Vector3f(1, color_image[1], color_image[2]);
      // 	// mymapping = color_image;
      // }else if(temperature[2] < 100){
      // 	mymapping = Eigen::Vector3f(color_image[0], color_image[1], 1);
      	
      // }else{
      // 	mymapping = color_image;
      // }

      (*colors)[track_id] += temperature; // old one that worked
      // (*colors)[track_id] += Eigen::Vector3f(mymapping[0] * 255, mymapping[1] * 255, mymapping[2] * 255);;
      // (*colors)[track_id] += fusion_thermal_rgb;
      // (*colors)[track_id] +=  255.0 * image.GetXY(x, y);;
      // (*colors)[track_id] += 255.0 * combinedIm.GetXY(x, y);



      
    }
/*  } else if (image.Channels() == 1) {
    for (const TrackId track_id : track_ids) {
      const Feature feature = *view.GetFeature(track_id);
      const int x = static_cast<int>(feature.x());
      const int y = static_cast<int>(feature.y());
      std::lock_guard<std::mutex> lock(*mutex_lock);
      (*colors)[track_id] += 255.0 * image.GetXY(x, y);
    }*/
  } else {
    LOG(FATAL) << "The image file at: " << image_file
               << " is not an RGB or a grayscale image so the color cannot be "
                  "extracted.";
  }
}



void myColorizeReconstruction(const std::string& image_directory,
                            const int num_threads,
                            Reconstruction* reconstruction) {
  CHECK(DirectoryExists(image_directory))
      << "The image directory " << image_directory << " does not exist.";
  CHECK_GT(num_threads, 0);
  CHECK_NOTNULL(reconstruction);

  // Initialize the colors to be (0, 0, 0).
  const auto& track_ids = reconstruction->TrackIds();
  std::unordered_map<TrackId, Eigen::Vector3f> colors;
  for (const TrackId track_id : track_ids) {
    colors[track_id].setZero();
  }

  // For each image, find the color of each feature and add the value to the
  // colors map.
  std::unique_ptr<ThreadPool> pool(new ThreadPool(num_threads));
  std::mutex mutex_lock;
  const auto& view_ids = reconstruction->ViewIds();
  for (const ViewId view_id : view_ids) {
    const View* view = reconstruction->View(view_id);
    const std::string image_filepath = image_directory + view->Name();
    std::string thermal_name = view->Name();
    const std::string image_filepath_thermal = "/TheiaSfM/iron/iron/thermal_translation/thermal" + thermal_name.substr(3,thermal_name.length());
    const std::string image_filepath_combined = "/TheiaSfM/iron/iron/fusion/img" + thermal_name.substr(3,thermal_name.length());
    // std::cout << image_filepath_thermal <<" ime" << std::endl;
    CHECK(FileExists(image_filepath)) << "The image file: " << image_filepath
                                      << " does not exist!";
    pool->Add(ExtractColorsFromImage,
              image_filepath,
              image_filepath_thermal,
              image_filepath_combined,
              *view,
              &colors,
              &mutex_lock);
  }
  // Wait for all threads to finish before proceeding.
  pool.reset(nullptr);

  // The colors map now contains a sum of all colors, so to get the mean we must
  // divide by the number of observations in each track.
  for (const TrackId track_id : track_ids) {
    Eigen::Vector3f color = FindOrDie(colors, track_id);

    Track* track = reconstruction->MutableTrack(track_id);
    color /= static_cast<float>(track->NumViews());
    *track->MutableColor() = color.cast<uint8_t>();
  }
}



int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  THEIA_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

  // Load the reconstuction.
  theia::Reconstruction reconstruction;
  CHECK(theia::ReadReconstruction(FLAGS_input_reconstruction_file,
                                  &reconstruction));

  myColorizeReconstruction(FLAGS_image_directory,
                                FLAGS_num_threads,
                                &reconstruction);

  CHECK(theia::WriteReconstruction(reconstruction,
                                   FLAGS_output_reconstruction_file));
  return 0;
}
