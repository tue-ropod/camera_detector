using namespace std;

/// [headers]

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "ed_gui_server/objsPosVel.h"
//#include "camera_detector/detections.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>
//#include <tf2_geometry_msgs>



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
DEFINE_bool(do_print,                 false,
            "Disable to disable the visual display.");
DEFINE_string(net_size,                 "256x176",
              "openpose net size (multiples of 16)");
DEFINE_string(model_folder,std::string("/home/nvidia/OpenPose/openpose/models/"),
              "absolute or relative path to openpose trained models");
DEFINE_bool(no_ROS,                 false,
            "Disable communication with RoPod");
// camera_detector headers
#include <camera_detector/kinectDetector.h>
/// [headers]




bool protonect_shutdown = false; ///< Whether the running application should shut down.

void sigint_handler(int s)
{
    protonect_shutdown = true;
}

bool protonect_paused = false;
libfreenect2::Freenect2Device *devtopause;

ed_gui_server::objsPosVel objsInfo_store; // create global object to store lrf info

void showData1(const ed_gui_server::objsPosVel& objsInfo)
{
    objsInfo_store=objsInfo;
}

void createCilinderMarkerLowerLegs(camera_detector::detections detectionData,int i, visualization_msgs::Marker &marker) {
    double dx,dy,dz;
    double t3,t2,t1;
    dx = detectionData.detections[i].xArray.back()-detectionData.detections[i].xArray[0];
    dy = detectionData.detections[i].yArray.back()-detectionData.detections[i].yArray[0];
    dz = detectionData.detections[i].zArray.back()-detectionData.detections[i].zArray[0];

    t1 = atan2(-dy,dz);
    t2 = asin(dx/detectionData.detections[i].d);
    t3 = 0.0;

    tf2::Quaternion angleQ;
    angleQ.setEuler(t2,t1,t3);
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = detectionData.detections[i].xArray[2]-rLeg;
    marker.pose.position.y = detectionData.detections[i].yArray[2];
    marker.pose.position.z = detectionData.detections[i].zArray[2];;

    marker.pose.orientation.x = angleQ.x();
    marker.pose.orientation.y = angleQ.y();
    marker.pose.orientation.z = angleQ.z();
    marker.pose.orientation.w = angleQ.w();
    marker.scale.x = detectionData.detections[i].r;
    marker.scale.y = detectionData.detections[i].r;
    marker.scale.z = detectionData.detections[i].d;
    if (detectionData.detections[i].associated) {
        marker.color.a = 0.8; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else if (!detectionData.detections[i].validHeight||!detectionData.detections[i].validSize||!detectionData.detections[i].validUniqueness) {//||!detectionData.detections[i].validContinuous) {
        marker.color.a = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else {
        marker.color.a = 0.4;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
    }
}

void createPointsMarkerA(camera_detector::detections detectionData,int i, int j, visualization_msgs::Marker &marker) {
    geometry_msgs::Point p;
    marker.header.stamp = ros::Time();
    marker.id = 1000+i;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Quaternion angleQ;
    angleQ.setEuler(0.0,0.0,0.0);
    marker.pose.orientation.x = angleQ.x();
    marker.pose.orientation.y = angleQ.y();
    marker.pose.orientation.z = angleQ.z();
    marker.pose.orientation.w = angleQ.w();
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    p.x = detectionData.detections[i].xArray[j]-rLeg;
    p.y = detectionData.detections[i].yArray[j];
    p.z = detectionData.detections[i].zArray[j];
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.points.push_back(p);
}

void createPointsMarkerVA(camera_detector::detections detectionData,int i, int j, visualization_msgs::Marker &marker) {
    geometry_msgs::Point p;
    marker.header.stamp = ros::Time();
    marker.id = 10000+i;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Quaternion angleQ;
    angleQ.setEuler(0.0,0.0,0.0);
    marker.pose.orientation.x = angleQ.x();
    marker.pose.orientation.y = angleQ.y();
    marker.pose.orientation.z = angleQ.z();
    marker.pose.orientation.w = angleQ.w();
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    p.x = detectionData.detections[i].xValArray[j]-rLeg;
    p.y = detectionData.detections[i].yValArray[j];
    p.z = detectionData.detections[i].zValArray[j];
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.points.push_back(p);
}

void createDiskMarker(camera_detector::detections detectionData, int i, visualization_msgs::Marker &marker) {
    tf2::Quaternion angleQ;
    angleQ.setEuler(0.0,0.0,0.0);
    marker.header.stamp = ros::Time();
    marker.id = 100+i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = detectionData.detections[i].x-rLeg;
    marker.pose.position.y = detectionData.detections[i].y;
    marker.pose.position.z = detectionData.detections[i].z;
    marker.pose.orientation.x = angleQ.x();
    marker.pose.orientation.y = angleQ.y();
    marker.pose.orientation.z = angleQ.z();
    marker.pose.orientation.w = angleQ.w();
    marker.scale.x = detectionData.detections[i].r+0.05;
    marker.scale.y = detectionData.detections[i].r+0.05;
    marker.scale.z = 0.02;
    marker.color.a = 0.6; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
}




/// [main]
int main(int argc, char *argv[])
/// [main]
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_no_display) {
        cv::imshow("esc",cv::Mat(1,1,CV_32FC1,1));
    }
    // instantiate processing functions
    detection_processing dp;
    // Start ROS listener
    ros::Publisher pub2, objDetections_pub;
    ros::Subscriber sub1;
    //    if (!FLAGS_no_ROS) {
    cout<<"Starting ROS listener..."<<endl;
    ros::init(argc, argv, "LRF_detections_listener");
    ros::NodeHandle n;
    //    tf2_ros::Buffer tfBuffer;
    tf::TransformListener listener_tf;
    sub1 = n.subscribe("/ropod_tue_2/ed/gui/objectPosVel", 3, showData1);
    //    sub2 = n.subscribe("tf", 1000, showData2);
    pub2 = n.advertise<camera_detector::detections>("rawCameraDetections",3);
    //    visualization_msgs::MarkerArray markerArray;
    objDetections_pub = n.advertise<visualization_msgs::MarkerArray>("/Jetson/cameraDetections",3);


    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker;
    marker.ns = "cameraDetections";
    marker.header.frame_id = "ropod_tue_2/Kinect/lowerLegs";
    marker.action = visualization_msgs::Marker::ADD;

    //    }
    camera_detector::detections detectionData;
    //    ed_gui_server::objsPosVel objsInfo;
    //    ed_gui_server::objsPosVel objsInfo2;
    //    geometry_msgs ropodPosition;
    //     geometry_msgs::TransformStamped transformStamped;
//    tf::StampedTransform transform;
    //     transformStamped = tfBuffer.lookupTransform("map", "ropod_tue_2/odom",ros::Time(2.0));
    cout<<"listen to transformation frames"<<endl;

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
    libfreenect2::Freenect2 freenect2;detectionData;
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

    cv::Mat rgbmat, depthmat, newrgbmat, rgbd2;
    double sum_freq,single_freq = 0;
    const string target_frame = "/ropod_tue_2/laser/scan";
    const string base_frame = "/map";
    geometry_msgs::PoseStamped pose_itf,pose_i; // create stamped poses for transformation of frames (pose with ros header)

    /// [loop start]
    while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
    {
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
        {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        /// [loop start]

        //        if (!FLAGS_no_ROS) {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0)); // Call ROS stream with lrf detections and let it sleep max x seconds until skipping
        try {
            for (int i=0;i<objsInfo_store.objects.size();i++) {
                pose_i.header = objsInfo_store.header; // store header
                pose_i.pose = objsInfo_store.objects[i].circle.pose; // store pose of circular object i
                pose_i.pose.orientation.w=1.0; // fix orientation of object (circle has no real orientation in this case)
                listener_tf.transformPose(target_frame,ros::Time(0.0),pose_i,base_frame,pose_itf); // transform frame
                objsInfo_store.objects[i].circle.pose.position.x = pose_itf.pose.position.x;// rotate with 90 deg
                objsInfo_store.objects[i].circle.pose.position.y = pose_itf.pose.position.y;
            }
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        // convert kinect data to cv mat, which is useable for openpose
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
        cv::resize(rgbmat, rgbmat, cv::Size(), 0.5, 0.5); // resize image, no effect on speed
        cv::cvtColor(rgbmat, newrgbmat, CV_BGRA2RGB);     // transform four channel to RGB (delete alpha layer)

        clock_t begin = clock();

        //! [registration]
        registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
        //! [registration]
        cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);
        cv::resize(rgbd2, rgbd2, cv::Size(), 0.5, 0.5); // resize image, no effect on speed


        auto datumProcessed = opWrapper.emplaceAndPop(newrgbmat);
        if (datumProcessed != nullptr)
        {
            if (!FLAGS_no_display) {
                dp.display(datumProcessed);            // plots keypoints on top of picture
//                cv::imshow("depth", depthmat / 4096.0f); // plots depth data in black/white picture
                cv::Mat flipImage;
                cv::flip(rgbd2, flipImage,1);
                cv::imshow("depth2RGBsize", flipImage / 4096.0f);
            }
            dp.processKeypoints(datumProcessed,rgbd2,detectionData,objsInfo_store);       // prints keypoints in command window
        }

        // Delete old visualization RVIZ topic
        double N;
        N = markerArray.markers.size();
        markerArray.markers.clear();

        visualization_msgs::MarkerArray deleteAllMarkerArray;
        visualization_msgs::Marker deleteAllMarker;
        deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
        for (int i=0;i<N;i++){
            deleteAllMarkerArray.markers.push_back(deleteAllMarker);
        }
        objDetections_pub.publish(deleteAllMarkerArray);

        // visualize detections in RVIZ topic
        for (int i=0;i<detectionData.detections.size();i++) {
            for (int j=0;j<detectionData.detections[i].xArray.size();j++) {
                createPointsMarkerA(detectionData, i, j, marker);
            }
            markerArray.markers.push_back(marker);
            marker.points.clear();
            for (int j=0;j<detectionData.detections[i].xValArray.size();j++) {
                createPointsMarkerVA(detectionData, i, j, marker);
            }
            markerArray.markers.push_back(marker);
            marker.points.clear();
            createCilinderMarkerLowerLegs(detectionData, i, marker);
            markerArray.markers.push_back(marker);
            createDiskMarker(detectionData, i, marker);
            markerArray.markers.push_back(marker);
	    cout<<"detection number: "<<i<<endl;
        }
        objDetections_pub.publish (markerArray);
        cout<<"published pose"<<endl;




        if (!FLAGS_no_ROS) {
            pub2.publish(detectionData);
            detectionData.detections.clear();
        }
        int key = cv::waitKey(1);
        if (key==27) { // stop program if esc key is pressed
            break;
        }        single_freq = 1/(double(clock()-begin)/CLOCKS_PER_SEC);
        sum_freq = sum_freq+single_freq;
        /// [Detect and display color stream]

        framecount++;

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

