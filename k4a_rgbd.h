#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <iostream> 
#include <string>
#include <iomanip>
#include <sstream>
#include "turbojpeg.h"
#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>

static int playback(char* input_path, const char *output_path = "capture");
k4a_image_t mjpg_to_bgra(k4a_image_t color_image);
void writeColor(k4a_image_t in, std::string name);
void writeDepth(k4a_image_t in, std::string name);
static void print_usage();