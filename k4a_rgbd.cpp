// k4a_rgbd.cpp : This file contains the 'main' function. Program execution begins and ends there.
//


#include "k4a_rgbd.h"

void writeColor(k4a_image_t in, std::string name) {
    k4a_image_format_t format = k4a_image_get_format(in); // K4A_IMAGE_FORMAT_COLOR_BGRA32 

    // get raw buffer
    uint8_t* buffer = k4a_image_get_buffer(in);

    // convert the raw buffer to cv::Mat
    int rows = k4a_image_get_height_pixels(in);
    int cols = k4a_image_get_width_pixels(in);
    cv::Mat colorMat(rows, cols, CV_8UC4, (void*)buffer, cv::Mat::AUTO_STEP);
    cv::imwrite(name, colorMat);
}

void writeDepth(k4a_image_t in, std::string name) {
    k4a_image_format_t format = k4a_image_get_format(in); // K4A_IMAGE_FORMAT_DEPTH_16U 

    // get raw buffer
    uint8_t* buffer = k4a_image_get_buffer(in);

    // convert the raw buffer to cv::Mat
    int rows = k4a_image_get_height_pixels(in);
    int cols = k4a_image_get_width_pixels(in);
    cv::Mat colorMat(rows, cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);
    cv::imwrite(name, colorMat);
}

static bool depth_to_color(k4a_transformation_t transformation_handle,
    const k4a_image_t depth_image,
    const k4a_image_t color_image,
    const std::string& name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }
    writeDepth(transformed_depth_image, name);
    k4a_image_release(transformed_depth_image);
    return true;
}

k4a_image_t mjpg_to_bgra(k4a_image_t color_image) {
    // Convert color frame from mjpeg to bgra
    k4a_image_t uncompressed_color_image = NULL;
    k4a_image_format_t format;
    format = k4a_image_get_format(color_image);
    if (format != K4A_IMAGE_FORMAT_COLOR_MJPG)
    {
        printf("Color format not supported. Please use MJPEG\n");
        return NULL;
    }

    int color_width, color_height;
    color_width = k4a_image_get_width_pixels(color_image);
    color_height = k4a_image_get_height_pixels(color_image);

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        color_width,
        color_height,
        color_width * 4 * (int)sizeof(uint8_t),
        &uncompressed_color_image))
    {
        printf("Failed to create image buffer\n");
        return NULL;
    }
    tjhandle tjHandle;
    tjHandle = tjInitDecompress();
    if (tjDecompress2(tjHandle,
        k4a_image_get_buffer(color_image),
        static_cast<unsigned long>(k4a_image_get_size(color_image)),
        k4a_image_get_buffer(uncompressed_color_image),
        color_width,
        0, // pitch
        color_height,
        TJPF_BGRA,
        TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
    {
        printf("Failed to decompress color frame\n");
        if (tjDestroy(tjHandle))
        {
            printf("Failed to destroy turboJPEG handle\n");
        }
        return NULL;
    }
    if (tjDestroy(tjHandle))
    {
        printf("Failed to destroy turboJPEG handle\n");
    }

    return uncompressed_color_image;

}

static int playback(char* input_path, const char *output_path)
{
    int returnCode = 1;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t uncompressed_color_image = NULL;
    std::string filename;
    std::string op(output_path);
    printf("Creating output path\n");
    cv::utils::fs::createDirectory(op);
    printf("Createing path for color images\n");
    std::string depth_dir = op + "\\depth";
    printf("Creating path for depth images\n");
    std::string color_dir = op + "\\color";
    cv::utils::fs::createDirectory(depth_dir);
    cv::utils::fs::createDirectory(color_dir);
    int count = 0;
    k4a_result_t result;
    k4a_stream_result_t stream_result;

    // Open recording
    printf("Opening recording\n");
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        goto Exit;
    }
    printf("Recording opened.\nCreating images now.\n");
    stream_result = K4A_STREAM_RESULT_SUCCEEDED;
    
    while (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
    {
        stream_result = k4a_playback_get_next_capture(playback, &capture);
        if (stream_result == K4A_STREAM_RESULT_SUCCEEDED)
        {
            // Process capture here
           
            if (count==0){
                if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
                {
                    printf("Failed to get calibration\n");
                    goto Exit;
                }

                transformation = k4a_transformation_create(&calibration);
            }

            // Fetch frame
            depth_image = k4a_capture_get_depth_image(capture);
            if (depth_image == 0)
            {
                printf("Failed to get depth image from capture\n");
                continue;
            }

            color_image = k4a_capture_get_color_image(capture);
            if (color_image == 0)
            {
                printf("Failed to get color image from capture\n");
                continue;
            }

            uncompressed_color_image = mjpg_to_bgra(color_image);
            if (uncompressed_color_image == NULL)
                goto Exit;
            std::stringstream filename;
            filename<< std::setfill('0') << std::setw(5) << count++;
            // Compute warping depth image into color camera geometry
            if (depth_to_color(transformation, depth_image, uncompressed_color_image, depth_dir + "\\" + filename.str() + ".png" ) == false)
            {
                printf("Failed to transform depth to color\n");
                goto Exit;
            }
            std::cout << filename.str() << std::endl;
            writeColor(uncompressed_color_image, color_dir + "\\" + filename.str() + ".jpg");
            returnCode = 0;


            k4a_capture_release(capture);
        }
        else if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            printf("Reached EOF\n");
            // End of file reached
            break;
        }
    }
    if (stream_result == K4A_STREAM_RESULT_FAILED)
    {
        printf("Failed to read entire recording\n");
        return -1;
    }
Exit:
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (depth_image != NULL)
    {
        k4a_image_release(depth_image);
    }
    if (color_image != NULL)
    {
        k4a_image_release(color_image);
    }
    if (uncompressed_color_image != NULL)
    {
        k4a_image_release(uncompressed_color_image);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }
    return returnCode;
}

static void print_usage()
{
    printf("Usage: k4a_rgbd.exe <filename.mkv> <output_folder>\n");
}

int main(int argc, char** argv)
{
    int returnCode = 0;
    printf("Opened MKV decoder\n");

    if (argc < 2)
    {
        print_usage();
    }
    else
    {
        printf("Opening Playback and decoding to depth / color images.\n");
        playback(argv[1],argv[2]);
    }

    return returnCode;
}
