#include <string> 
#include <fstream>   

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include <cartographer_ros_msgs/SubmapList.h>
#include <math.h>   
#include <vector>
#include <iomanip>
#include <jpeglib.h>

static bool start_flag = false;

static int cnt = 0;
static std::string path = "/home/fan/ros_ws/trim/mappng/";

void check_submap_list(const cartographer_ros_msgs::SubmapListConstPtr& submap_list){
  if (start_flag) return;

  start_flag = true;
}

void save_image(const std::vector<uint8_t>& buffer, std::string strPath, int nImageWidth, int nImageHeight,int nQuality){

    FILE* outfile = fopen(strPath.c_str(), "wb");

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = nImageWidth;
    cinfo.image_height = nImageHeight;
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, nQuality, true);
    jpeg_start_compress(&cinfo, true);

    JSAMPROW row_pointer;

    while (cinfo.next_scanline < cinfo.image_height){
        row_pointer  = (JSAMPROW) &buffer[(nImageHeight - cinfo.next_scanline -1) * nImageWidth];
        jpeg_write_scanlines(&cinfo, &row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    fclose(outfile);
    jpeg_destroy_compress(&cinfo);
}

void callback(const nav_msgs::OccupancyGridConstPtr& data){


	static int w, h, offset_x, offset_y;
	static float resolution, origin_x, origin_y, target_image_size, scale, factor, map_resolution, map_origin_x, map_origin_y;
	// static int w, h;
	// static float resolution, origin_x, origin_y;

	std::string imagePath, yamlPath;
	// std::string imagePath;

	if (!start_flag) return;

	w = (*data).info.width;
	h = (*data).info.height;
	resolution = (*data).info.resolution;
	origin_x = (*data).info.origin.position.x;
	origin_y = (*data).info.origin.position.y;

	std::ostringstream temp;
	temp<<std::setw(4)<<std::setfill('0')<<cnt;

	imagePath = path + std::to_string(cnt) +".jpg";
	yamlPath = path + std::to_string(cnt) +".yaml";

	offset_x = int(origin_x/resolution);
	offset_y = int(origin_y/resolution);

	target_image_size = 1000;
	scale = target_image_size;
	factor = 1;

	while(1){
	if ((-offset_x < scale/2) && (-offset_y < scale/2) && (w+offset_x < scale/2) && (h+offset_y < scale/2)) 
			break;	
		scale = scale * 2;
		factor = factor * 2;	
	}

	map_resolution = resolution;
	map_origin_x = origin_x;
	map_origin_y = origin_y;

	std::ofstream LogWriter;
	LogWriter.open(yamlPath);	
	LogWriter<<"free_thresh: 0.196\n";
	LogWriter<<"image: " + imagePath + "\n";
	LogWriter<<"negate: 0\n";
	LogWriter<<"occupied_thresh: 0.65\n";
	LogWriter<<"origin:\n";
	LogWriter<<"- " + std::to_string(map_origin_x) + "\n";
	LogWriter<<"- " + std::to_string(map_origin_y) + "\n";
	LogWriter<<"- " + std::to_string(0.0) + "\n";
	LogWriter<<"resolution: "+ std::to_string(map_resolution);
	LogWriter.close();

  std::vector<uint8_t> cdata_np(data->data.begin(), data->data.end());

  for (size_t i=0; i < cdata_np.size(); i++){
      if (cdata_np[i] == 255) {
          cdata_np[i] = 50;
      }

      float temp = (float)cdata_np[i]*2.55;       
      cdata_np[i] = ~(uint8_t)((unsigned)temp <= 255 ? temp : temp > 0 ? 255 : 0); 
  }

  save_image(cdata_np, imagePath, w, h, 100);

	// cnt = (cnt+1)%5;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "map_store");
	
	ros::NodeHandle nh;

	ros::Subscriber submap_list_sub = nh.subscribe("submap_list", 1, &check_submap_list);		

	ros::Subscriber occupancyGrid_sub = nh.subscribe("map", 1, &callback);	

	ros::spin();
	return 0;
}

