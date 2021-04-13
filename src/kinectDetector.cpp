using namespace std;

/// [headers]

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
// Kinect drivers
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>


// Openpose API (including opencv)
#define OPENPOSE_FLAGS_DISABLE_PRODUCER
#define OPENPOSE_FLAGS_DISABLE_DISPLAY
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>


// Custom OpenPose flags
DEFINE_bool(no_display,                 false,
            "Enable to disable the visual display.");
DEFINE_bool(saveDetectionData,          false,
            "Enable to store the detection data.");
DEFINE_bool(do_print,                 false,
            "Disable to disable the visual display.");
DEFINE_string(net_size,                 "288x160",
              "openpose net size (multiples of 16)");
DEFINE_string(model_folder,std::string("/home/nvidia/OpenPose/openpose/models/"),
              "absolute or relative path to openpose trained models");
DEFINE_bool(no_ROS,                 false,
            "Disable communication with RoPod");
// camera_detector headers
#include <camera_detector/kinectDetector.h>
/// [headers]

#define CAMERA_UPSIDEDOWN


bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

/// [main]
int main(int argc, char *argv[])
/// [main]
{
//    camera_detector::persons persons;
    gflags::ParseCommandLineFlags(&argc, &argv, true);
//    if (FLAGS_no_display) {
//        cv::imshow("esc",cv::Mat(1,1,CV_32FC1,1));
//    }
    // instantiate processing functions
    detection_processing dp;
//    // Start ROS listener
    //    ros::Publisher pub2, objDetections_pub, objAssociations_pub, objpersons_pub;
      ros::Publisher objDetections_pub, indx_pub, indy_pub;
//    ros::Subscriber sub1;
//    //    if (!FLAGS_no_ROS) {
//    cout<<"Starting ROS listener..."<<endl;
    ros::init(argc, argv, "LRF_detections_listener");
    ros::NodeHandle n;
//    //    tf2_ros::Buffer tfBuffer;
//    tf::TransformListener listener_tf;
//    sub1 = n.subscribe("/ropod_tue_2/ed/gui/objectPosVel", 3, showData1);
////    sub2 = n.subscribe("/clock", 3, showData2);
//    pub2 = n.advertise<camera_detector::detections>("rawCameraDetections",3);
//    //    visualization_msgs::MarkerArray markerArray;
    objDetections_pub = n.advertise<hip_msgs::detections>("/Jetson/cameraDetections",3);
    indx_pub = n.advertise<std_msgs::Float32>("/Jetson/indx",3);
    indy_pub = n.advertise<std_msgs::Float32>("/Jetson/indy",3);
//    objAssociations_pub = n.advertise<visualization_msgs::MarkerArray>("/Jetson/LRFAssociations",3);
//    objpersons_pub = n.advertise<camera_detector::persons>("/Jetson/persons",3);
    std_msgs::Float32 indxMP;
    std_msgs::Float32 indyMP;

//    visualization_msgs::MarkerArray markerArrayA;
//    visualization_msgs::Marker markerA;
//    markerA.ns = "LRFAssociations";
//    markerA.header.frame_id = "ropod_tue_2/laser/scan";
//    markerA.action = visualization_msgs::Marker::ADD;

//    visualization_msgs::MarkerArray markerArray;
//    visualization_msgs::Marker marker;
//    marker.ns = "cameraDetections";
//    marker.header.frame_id = "ropod_tue_2/Kinect/lowerLegs";
//    marker.action = visualization_msgs::Marker::ADD;

//    //    }
    hip_msgs::detections detectionData;
//    //    ed_gui_server::objsPosVel objsInfo;
//    //    ed_gui_server::objsPosVel objsInfo2;
//    //    geometry_msgs ropodPosition;
//    //     geometry_msgs::TransformStamped transformStamped;
////    tf::StampedTransform transform;
//    //     transformStamped = tfBuffer.lookupTransform("map", "ropod_tue_2/odom",ros::Time(2.0));
//    cout<<"listen to transformation frames"<<endl;

    //    while (n.ok()) {

    //    }


    op::log("Starting OpenPose...", op::Priority::High);
    // Configuring OpenPose
    op::log("Configuring OpenPose...", op::Priority::High);
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    // logging_level
    op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
              __LINE__, __FUNCTION__, __FILE__);
    op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
    op::Profiler::setDefaultX(FLAGS_profile_speed);

    // netInputSize
    const auto netInputSize = op::flagsToPoint(FLAGS_net_size);  // important speed/accuracy variable (multiple of 16, default -1)
    // poseMode
    const auto poseMode = op::flagsToPoseMode(FLAGS_body);
    // Pose configuration
    const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
    const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
    const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
    const auto outputSize = op::flagsToPoint(FLAGS_output_resolution);
    const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
                                                  FLAGS_heatmaps_add_PAFs);
    const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
    FLAGS_maximize_positives = true;
    const op::WrapperStructPose wrapperStructPose{poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
                FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
                poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
                FLAGS_part_to_show, FLAGS_model_folder, heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
                (float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives};

    opWrapper.configure(wrapperStructPose);

    // Starting OpenPose
    op::log("Starting thread(s)...", op::Priority::High);
    opWrapper.start();

    std::string program_path(argv[0]);
    std::cerr << "Version: " << LIBFREENECT2_VERSION << std::endl;
    size_t executable_name_idx = program_path.rfind("Protonect");

    std::string binpath = "/";

    if(executable_name_idx != std::string::npos)
    {
        binpath = program_path.substr(0, executable_name_idx);
    }

    /// [context]
    libfreenect2::Freenect2 freenect2;//detectionData;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    /// [context]

    std::string serial = "";

    bool viewer_enabled = true;
    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;
    size_t framemax = -1;

#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    if(!pipeline)
        pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
#else
    std::cout << "CUDA pipeline is not supported!" << std::endl;
#endif
    for(int argI = 1; argI < argc; ++argI)
    {
        const std::string arg(argv[argI]);

        if(arg == "-help" || arg == "--help" || arg == "-h" || arg == "-v" || arg == "--version" || arg == "-version")
        {
            // Just let the initial lines display at the beginning of main
            return 0;
        }
        else if(arg.find("-gpu=") == 0)
        {
            if (pipeline)
            {
                std::cerr << "-gpu must be specified before pipeline argument" << std::endl;
                return -1;
            }
            deviceId = atoi(argv[argI] + 5);
        }
        else if(arg == "cpu")
        {
            if(!pipeline)
                /// [pipeline]
                pipeline = new libfreenect2::CpuPacketPipeline();
            /// [pipeline]
        }
        else if(arg == "gl")
        {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
            std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "cl")
        {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenCLPacketPipeline(deviceId);
#else
            std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "clkde")
        {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::OpenCLKdePacketPipeline(deviceId);
#else
            std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg == "cudakde")
        {
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
            if(!pipeline)
                pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
#else
            std::cout << "CUDA pipeline is not supported!" << std::endl;
#endif
        }
        else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
        {
            serial = arg;
        }
        else if(arg == "-noviewer" || arg == "--noviewer")
        {
            viewer_enabled = false;
        }
        else if(arg == "-norgb" || arg == "--norgb")
        {
            enable_rgb = false;
        }
        else if(arg == "-nodepth" || arg == "--nodepth")
        {
            enable_depth = false;
        }
        else if(arg == "-frames")
        {
            ++argI;
            framemax = strtol(argv[argI], NULL, 0);
            if (framemax == 0) {
                std::cerr << "invalid frame count '" << argv[argI] << "'" << std::endl;
                return -1;
            }
        }
        else
        {
            std::cout << "Unknown argument: " << arg << std::endl;
        }
    }

    /// [discovery]
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    /// [discovery]

    if(pipeline)
    {
        /// [open]
        dev = freenect2.openDevice(serial, pipeline);
        /// [open]
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }

    devtopause = dev;

    protonect_shutdown = false;

    /// [listeners]
    int types = 0;
    types |= libfreenect2::Frame::Color;
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    /// [listeners]

    /// [start]
    dev->start();
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    /// [start]

    size_t framecount = 0;

    //! [registration setup]
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), depth2rgb(1920, 1080 + 2, 4); // check here (https://github.com/OpenKinect/libfreenect2/issues/337) and here (https://github.com/OpenKinect/libfreenect2/issues/464) why depth2rgb image should be bigger
    //! [registration setup]

    cv::Mat rgbmat, depthmat, newrgbmat, rgbd2, rgbmatTest, depthmatTest, depthMathConverted;
    double sum_freq,single_freq = 0;
//    const string target_frame = "/ropod_tue_2/laser/scan";
//    const string base_frame = "/map";
//    geometry_msgs::PoseStamped pose_itf,pose_i; // create stamped poses for transformation of frames (pose with ros header)

//    ofstream storefile;
//    string fileName= "associationData" + IDMeasurement + ".csv";
//    storefile.open(fileName);
//    int time_count=0;
int cntr = 0;
    /// [loop start]
    while(n.ok())
    {
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
        {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        /// [loop start]


        // convert kinect data to cv mat, which is useable for openpose
#ifdef CAMERA_UPSIDEDOWN
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmatTest);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmatTest);

        // Rotate when kinect upside down
        cv::flip(rgbmatTest, rgbmat, -1);
        cv::flip(depthmatTest, depthmat, -1);
#else
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
#endif


        cv::resize(rgbmat, rgbmat, cv::Size(), 0.25, 0.25); // resize image, no effect on speed
        cv::cvtColor(rgbmat, newrgbmat, CV_BGRA2RGB);     // transform four channel to RGB (delete alpha layer)

//        cv::Mat flipImageTest;
//        cv::flip(rgbmatTest, rgbmat,0);
//        cv::imshow("depth2RGBsize", flipImageTest / 4096.0f);

        clock_t begin = clock();

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
        cv::resize(rgbd2, rgbd2, cv::Size(), 0.25, 0.25); // resize image, no effect on speed

        auto datumProcessed = opWrapper.emplaceAndPop(newrgbmat);
        if (datumProcessed != nullptr)
        {
            ros::Time stamp = ros::Time::now();
            if (!FLAGS_no_display) {
                dp.display(datumProcessed,stamp.toNSec());            // plots keypoints on top of picture
              usleep(0.0001);

               // cv::imshow("depth", depthmat / 4096.0f); // plots depth data in black/white picture
//		depthmat.convertTo(depthMathConverted,CV_8UC1);
//                cv::imwrite("/home/nvidia/pictures/depth"+to_string(cntr)+ ".jpg", depthMathConverted); // plots depth data in black/white picture
cntr++;


//                cv::Mat flipImage;
#ifdef CAMERA_UPSIDEDOWN
                cv::flip(rgbd2, rgbd2,-1);
#endif

//                cv::imwrite("/home/nvidia/pictures/fliprgbd2"+to_string(cntr)+ ".jpg", rgbd2/4096.0f); // lots depth data in black/white picture

//                cv::imshow("depth2RGBsize", flipImage / 4096.0f);
              int key = cv::waitKey(1);
              if (key==27) { // stop program if esc key is pressed
                  break;
              }
            }
//            persons.persons.clear();
            // TODO check if writing to file required
            detectionData.header.stamp.nsec = stamp.nsec;
            detectionData.header.stamp.sec = stamp.sec;
            detectionData.header.frame_id = "/Jetson";
            if (FLAGS_saveDetectionData) {
                dp.processKeypoints(datumProcessed,rgbd2,detectionData, stamp.toSec(), true);
            } else {
                dp.processKeypoints(datumProcessed,rgbd2,detectionData, stamp.toSec(), false);
            }

                   // prints keypoints in command window
        }



//        if (!FLAGS_no_ROS) {
        if (detectionData.detections.size()>0) {
            objDetections_pub.publish(detectionData);
            detectionData.detections.clear();
        }
//        }
        single_freq = 1/(double(clock()-begin)/CLOCKS_PER_SEC);
        sum_freq = sum_freq+single_freq;
        /// [Detect and display color stream]

        framecount++;
//        time_count++;

        if (framecount > 2*single_freq ) { // every two seconds put a message at the console
            std::cout<<"OpenPose detection at: "<<sum_freq/10<<" Hertz."<<std::endl;
            sum_freq = 0;
            framecount = 0;
            if (FLAGS_no_display) {
                std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
                listener.release(frames);
                continue;
            }
        }


        /// [loop end]
        listener.release(frames);
    }
    /// [loop end]

    /// [stop]
    dev->stop();
    dev->close();
    /// [stop]

    return 0;
}

