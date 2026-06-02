#include "safe_takeoff/image_processor.h"

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include<iostream>
#include <vector>

namespace SafeTakeoff{


  ImageProcessor::ImageProcessor(){
    hole=rs2::hole_filling_filter(1);
    thr_filter=rs2::threshold_filter(0.,5.);
    depth_to_disparity=rs2::disparity_transform(true);
    disparity_to_depth=rs2::disparity_transform(false);

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_device(serial_no);
    pipe.start(cfg);

    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    filters.push_back(thr_filter);
    filters.push_back(depth_to_disparity);
    filters.push_back(spat_filter);
    filters.push_back(temp_filter);
    filters.push_back(hole);
    filters.push_back(disparity_to_depth);

  }

  float ImageProcessor::dist_3d(const rs2::depth_frame& frame, int x,int y)
  {
      float point[3]; // From point (in 3D)
      float pixle[2];
      pixle[0]=x;
      pixle[1]=y;

      auto dist=frame.get_distance(x,y);
      rs2_intrinsics intr=frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
      rs2_deproject_pixel_to_point(point, &intr, pixle, dist);

      return point[2];
  }

  std::vector<std::vector<double> > ImageProcessor::process(){
    rs2::frameset data = pipe.wait_for_frames();
    rs2::frame depth_frame = data.get_depth_frame();
    if (!depth_frame)
        return std::vector<std::vector<double> >();

    rs2::frame filtered = depth_frame;

    for (auto& filter : filters)
        filtered = filter.process(filtered);

    w = filtered.as<rs2::video_frame>().get_width();
    h = filtered.as<rs2::video_frame>().get_height();

    std::vector<std::vector<double> >mesh;
    mesh.resize(YS);
    for(int y=0; y<YS; ++y)
        for(int x=0; x<XS; ++x){
            auto d=dist_3d(filtered,x,y);
            mesh[y].push_back(d);
        }

    return mesh;
  }



}
