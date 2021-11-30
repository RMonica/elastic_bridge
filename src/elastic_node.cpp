/*
 * Copyright (c) 2017-2019, Andrea Pagani, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <init_fake_opengl_context/fake_opengl_context.h>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <elastic_bridge/downloadPointcloud2Action.h>
#include <elastic_bridge/FrameState.h>

//c++
#include <vector>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

//ElasticFusion
#include <ElasticFusion.h>
#include "GlobalModel.h"
#include "IndexMap.h"
#include "GPUTexture.h"
#include "pangolin/pangolin.h"

//Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

//Pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

class ElasticBridge {
public:
    typedef uint32_t uint32;
    typedef uint16_t uint16;
    typedef uint8_t uint8;
    typedef uint64_t uint64;
    typedef std::vector<uint64> Uint64Vector;
    typedef std::vector<uint32> Uint32Vector;
    typedef message_filters::Subscriber<sensor_msgs::Image> ImageFilterSubscriber;
    typedef boost::shared_ptr<ImageFilterSubscriber> ImageFilterSubscriberPtr;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ATSyncPolicy;
    typedef message_filters::Synchronizer<ATSyncPolicy> ATSynchronizer;
    typedef boost::shared_ptr<ATSynchronizer> ATSynchronizerPtr;

private:
    int m_frame_count = 0;
    int m_height, m_width;
    float m_center_x,m_center_y,m_focal_x,m_focal_y;
    std::string m_TopicImageColor, m_TopicImageDepth, m_TopicCameraInfo,
      m_topic_periodic_world_pub,
      m_world_frame, m_camera_frame, m_input_world_frame, m_input_camera_frame;
    bool m_cameraInfoOK = false;
    bool m_poseFromTFAlways = false;
    bool m_poseFromTFFirst = false;
    Eigen::Matrix4f first_pose;
    
    ElasticFusion * m_eFusion = NULL;

    boost::mutex m_mutex;
    boost::condition_variable m_cond_var;

    uint32 m_download_request = false;
    uint32 m_download_ready = false;

    sensor_msgs::ImageConstPtr m_image_color; // NULL if not ready
    sensor_msgs::ImageConstPtr m_image_depth; // NULL if not ready
    sensor_msgs::CameraInfoConstPtr m_camera_info; // NULL if not ready
    
    bool m_started = true;

    bool m_black_to_nan = false;

    sensor_msgs::PointCloud2ConstPtr m_point2;
    Uint64Vector m_guids;
    Uint32Vector m_luids;

    ros::NodeHandle m_nh;
    ros::Subscriber cameraInfo_sub, scanReady_sub, scanFinish_sub;
    ros::Publisher m_periodic_cloud_pub, m_image_pub, m_guid_image_pub;
    ros::Publisher m_frame_state_stable_pub;
    tf::TransformBroadcaster m_trasformBroadcaster;
    tf::TransformListener m_trasformListener;
    ImageFilterSubscriberPtr m_imageColor_sub, m_imageDepth_sub;
    ATSynchronizerPtr m_sync_sub;

    std::string m_display_name;

    uint32 m_periodic_world_publication;

    std::vector<uint64> m_guid_assoc;
    uint64 m_guid_counter;

public:

    void sendTF(const Eigen::Matrix4f & pose, std::string from ,std::string to) {
        tf::Transform transform;
        Eigen::Affine3d affine;
        affine.matrix() = pose.cast<double>();
        tf::transformEigenToTF(affine, transform);
        const ros::Time & time = ros::Time::now();
        m_trasformBroadcaster.sendTransform(tf::StampedTransform(transform, time, from, to));
    }

    Eigen::Matrix4f readTF() {
        tf::StampedTransform transform;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        try {
            m_trasformListener.lookupTransform(m_input_camera_frame, m_input_world_frame, ros::Time(0), transform);
            Eigen::Affine3d affine;
            tf::transformTFToEigen(transform, affine);
            pose = affine.matrix().cast<float>();

        } catch (tf::TransformException &ex) {
            ROS_ERROR("elastic_bridge: could not read input TF: %s", ex.what());
        }
        return pose;
    }

    void publishFrameState(const std::vector<uint16> & depth_data, const std::vector<uint8> & rgb_data,
                           pangolin::GlTexture * guid_texture,
                           pangolin::GlTexture * image_texture, pangolin::GlTexture * vertex_texture,
                           pangolin::GlTexture * normal_texture,const Eigen::Affine3f & pose,
                           const std::vector<uint32> & disposed_luids,
                           ros::Publisher & pub)
    {
      if (pub.getNumSubscribers() == 0)
        return;

      const uint64 size = m_width * m_height;

      std::vector<uint32> guid_vec(size, 127);
#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
      guid_texture->Download(guid_vec.data(), GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_INT);
#endif

      std::vector<float> image_vec(size * 4, 127);
      image_texture->Download(image_vec.data(), GL_BGRA, GL_FLOAT);

      std::vector<float> vertex_vec(size * 4, 127);
      vertex_texture->Download(vertex_vec.data(), GL_RGBA, GL_FLOAT);

      std::vector<float> normal_vec(size * 4, 127);
      normal_texture->Download(normal_vec.data(), GL_RGBA, GL_FLOAT);

      elastic_bridge::FrameState state;

      state.width = m_width;
      state.height = m_height;
      state.seq = m_frame_count;

      state.focal_x = m_focal_x;
      state.center_x = m_center_x;
      state.focal_y = m_focal_y;
      state.center_y = m_center_y;

      state.input_color.resize(size * 3);
      state.input_depth.resize(size);
      state.color.resize(size * 3);
      state.position.resize(size * 3);
      state.depth.resize(size);
      state.luid.resize(size);
      state.guid.resize(size);
      state.normal.resize(size * 3);
      state.radius.resize(size);
      state.luid_removed = disposed_luids;
      for (uint64 i = 0; i < size; i++)
      {
        state.input_color[i * 3 + 2] = rgb_data[i * 3 + 0];
        state.input_color[i * 3 + 1] = rgb_data[i * 3 + 1];
        state.input_color[i * 3 + 0] = rgb_data[i * 3 + 2];

        state.input_depth[i] = depth_data[i];

        state.color[i * 3 + 0] = image_vec[i * 4 + 0];
        state.color[i * 3 + 1] = image_vec[i * 4 + 1];
        state.color[i * 3 + 2] = image_vec[i * 4 + 2];

        state.position[i * 3 + 0] = vertex_vec[i * 4 + 0];
        state.position[i * 3 + 1] = vertex_vec[i * 4 + 1];
        state.position[i * 3 + 2] = vertex_vec[i * 4 + 2];

        state.normal[i * 3 + 0] = normal_vec[i * 4 + 0];
        state.normal[i * 3 + 1] = normal_vec[i * 4 + 1];
        state.normal[i * 3 + 2] = normal_vec[i * 4 + 2];
        state.radius[i] = normal_vec[i * 4 + 3];

        state.depth[i] = vertex_vec[i * 4 + 2];
#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
        const uint32 luid = guid_vec[i];
#else
        const uint32 luid = 0;
#endif
        state.luid[i] = luid;
        state.guid[i] = luid ? m_guid_assoc[luid - 1] : 0;
      }
      state.max_luid = m_guid_assoc.size();

      tf::poseEigenToMsg(pose.cast<double>(),state.pose);

      pub.publish(state);
    }

    void imagePublish(const Eigen::Matrix4f & currPose,
                      const std::vector<uint16> & depth_data,const std::vector<uint8> & rgb_data) {

        const int n_pixel = m_height * m_width;

        pangolin::GlTexture * image_texture = m_eFusion->getIndexMap().imageTex()->texture;

#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
        pangolin::GlTexture * guid_texture = m_eFusion->getIndexMap().guidTex()->texture;
#else
        pangolin::GlTexture * guid_texture = NULL;
#endif

        pangolin::GlTexture * vertex_texture = m_eFusion->getIndexMap().vertexTex()->texture;

        pangolin::GlTexture * normal_texture = m_eFusion->getIndexMap().normalTex()->texture;

        std::vector<float> image_vec(n_pixel * 4, 127);
        image_texture->Download(image_vec.data(), GL_BGRA, GL_FLOAT);

        std::vector<uint32> guid_vec(n_pixel, 127);
#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
        guid_texture->Download(guid_vec.data(), GL_LUMINANCE_INTEGER_EXT, GL_UNSIGNED_INT);
#endif

        std::vector<float> bimage_vec(n_pixel * 4, 127);

#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
        const std::vector<GLuint> disposed_guids = m_eFusion->getGlobalModel().downloadDisposedGuids(true);
#else
        const std::vector<GLuint> disposed_guids;
#endif

        // scope only
        {

          const uint32 TEXTURE_PIXEL_SIZE = 4;
          const uint32 OUTPUT_PIXEL_SIZE = 4;

          sensor_msgs::Image image_msg;
          image_msg.height = m_height;
          image_msg.width = m_width;
          image_msg.encoding = "rgba8";
          image_msg.is_bigendian = false;
          image_msg.step = image_msg.width * OUTPUT_PIXEL_SIZE;
          image_msg.data.resize(n_pixel * OUTPUT_PIXEL_SIZE);
          for (int y = 0; y < m_height; y++)
              for (int x = 0; x < m_width; x++) {
                  const int i = y * m_width + x;

                  image_msg.data[i * OUTPUT_PIXEL_SIZE + 0] = image_vec[i * TEXTURE_PIXEL_SIZE + 0] * 255;
                  image_msg.data[i * OUTPUT_PIXEL_SIZE + 1] = image_vec[i * TEXTURE_PIXEL_SIZE + 1] * 255;
                  image_msg.data[i * OUTPUT_PIXEL_SIZE + 2] = image_vec[i * TEXTURE_PIXEL_SIZE + 2] * 255;
                  image_msg.data[i * OUTPUT_PIXEL_SIZE + 3] = 255;
              }

          m_image_pub.publish(image_msg);
        }

#ifdef ELASTIC_BRIDGE_EXTENSION_GUID
        // scope only
        {
          sensor_msgs::Image image_msg;
          image_msg.height = m_height;
          image_msg.width = m_width;
          image_msg.encoding = "rgba8";
          image_msg.is_bigendian = false;
          image_msg.step = image_msg.width * 4;
          image_msg.data.resize(n_pixel * 4);
          for (int y = 0; y < m_height; y++)
              for (int x = 0; x < m_width; x++) {
                  const int i = y * m_width + x;

                  image_msg.data[i * 4 + 0] = (guid_vec[i] >>  0) % 256;
                  image_msg.data[i * 4 + 1] = (guid_vec[i] >>  8) % 256;
                  image_msg.data[i * 4 + 2] = (guid_vec[i] >> 16) % 256;
                  image_msg.data[i * 4 + 3] = (guid_vec[i] >> 24) % 256;
              }

          m_guid_image_pub.publish(image_msg);
        }
#endif

        // scope only
        { 
            for (uint64 i = 0; i < guid_vec.size(); i++) {
                const uint32 internal_guid = guid_vec[i];
                if (!internal_guid)
                    continue;

                if (internal_guid > m_guid_assoc.size())
                    m_guid_assoc.resize(internal_guid,0);

                if (m_guid_assoc[internal_guid - 1] == 0)
                    m_guid_assoc[internal_guid - 1] = ++m_guid_counter;
            }

            for (uint64 i = 0; i < disposed_guids.size(); i++) {
                const uint32 disposed_guid = disposed_guids[i];
                if (!disposed_guid)
                    continue;

                if (disposed_guid > m_guid_assoc.size())
                    continue;

                m_guid_assoc[disposed_guid - 1] = 0;
            }

            Eigen::Affine3f pose;
            pose.matrix() = currPose;

            publishFrameState(depth_data,rgb_data,
                              guid_texture,image_texture,vertex_texture,normal_texture,
                              pose,disposed_guids,m_frame_state_stable_pub);
        }
    }
    
    sensor_msgs::PointCloud2ConstPtr requestDownload(Uint64Vector & guids,Uint32Vector & luids) {
        boost::mutex::scoped_lock lock(m_mutex);

        // wait until previous requests are satisfied
        while (m_download_request && !ros::isShuttingDown()) {
            m_cond_var.wait(lock);
        }

        m_download_request = true;
        m_cond_var.notify_all();

        if (ros::isShuttingDown())
          return m_point2;

        // wait until this request is satisfied
        while (!m_download_ready && !ros::isShuttingDown()) {
            m_cond_var.wait(lock);
        }

        m_download_ready = false;
        m_download_request = false;
        sensor_msgs::PointCloud2ConstPtr point2 = m_point2;
        guids.swap(m_guids);
        luids.swap(m_luids);
        m_guids.clear();
        m_luids.clear();
        m_point2.reset();
        m_cond_var.notify_all();

        return point2;
    }

    sensor_msgs::PointCloud2ConstPtr getPC2(Uint64Vector * guids = NULL,Uint32Vector * luids = NULL) {

        if (!m_eFusion) {
          sensor_msgs::PointCloud2Ptr pointCloud2ptr(new sensor_msgs::PointCloud2);
          pcl::PointCloud<pcl::PointSurfel> pcl_cloud;
          pcl::toROSMsg(pcl_cloud, *pointCloud2ptr);
          pointCloud2ptr->header.frame_id = m_world_frame;
          ROS_INFO("elastic_bridge: elastic fusion not initialized, returning empty surfel cloud.");
          return pointCloud2ptr;
        }

        Eigen::Vector4f * mapData = m_eFusion->getGlobalModel().downloadMap();

        const uint64 last_count = m_eFusion->getGlobalModel().lastCount();

        pcl::PointCloud<pcl::PointSurfel> pcl_cloud;
        pcl_cloud.is_dense = true;
        pcl_cloud.height = 1;
        pcl_cloud.reserve(last_count);

        if (guids) {
            guids->clear();
            guids->reserve(last_count);
        }

        if (luids) {
            luids->clear();
            luids->reserve(last_count);
        }

        const float confidenceThreshold = m_eFusion->getConfidenceThreshold();

        int count = 0;
        ROS_INFO("elastic_bridge: downloading %d points.", int(last_count));
        for (unsigned int i = 0; i < last_count; i++) {
            Eigen::Vector4f pos = mapData[(i * 3) + 0];

            if(pos[3] <= confidenceThreshold)
              continue;

            pcl::PointSurfel pointSurfel;

            Eigen::Vector4f col = mapData[(i * 3) + 1];
            Eigen::Vector4f nor = mapData[(i * 3) + 2];

            nor[0] *= -1;
            nor[1] *= -1;
            nor[2] *= -1;

            pointSurfel.x = pos[0];
            pointSurfel.y = pos[1];
            pointSurfel.z = pos[2];

            unsigned char b = int(col[0]) >> 16 & 0xFF;
            unsigned char g = int(col[0]) >> 8 & 0xFF;
            unsigned char r = int(col[0]) & 0xFF;

            pointSurfel.r = r;
            pointSurfel.g = g;
            pointSurfel.b = b;
            pointSurfel.a = 255;
            pointSurfel.normal_x = nor[0];
            pointSurfel.normal_y = nor[1];
            pointSurfel.normal_z = nor[2];
            pointSurfel.radius = nor[3];

            pcl_cloud.points.push_back(pointSurfel);

            const uint32 internal_guid = col[1];
            const uint64 guid = !internal_guid ? 0 : m_guid_assoc[internal_guid - 1];
            if (guids)
                guids->push_back(guid);
            if (luids)
                luids->push_back(internal_guid);

            count++;
        }

        delete [] mapData;

        pcl_cloud.width = count;

        ROS_INFO("elastic_bridge: sending %d points.", int(count));

        sensor_msgs::PointCloud2Ptr pointCloud2ptr(new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2 & pointCloud2 = *pointCloud2ptr;
        pcl::toROSMsg(pcl_cloud, pointCloud2);
        pointCloud2.header.frame_id = m_world_frame;

        return pointCloud2ptr;
    }

    void publishWorldExec() {
        sensor_msgs::PointCloud2ConstPtr pointCloud2 = this->getPC2();
        m_periodic_cloud_pub.publish(pointCloud2);
    }

    void scanReadyCallback(const std_msgs::EmptyConstPtr& msg) {
        ROS_INFO("elastic_bridge: started.");
        boost::mutex::scoped_lock lock(m_mutex);
        m_started = true;
    }

    void scanFinishCallback(const std_msgs::EmptyConstPtr& msg) {
        ROS_INFO("elastic_bridge: stopped.");
        boost::mutex::scoped_lock lock(m_mutex);
        m_started = false;
    }

    void cameraInfoCallbackWorker(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (m_cameraInfoOK)
          return; // already initialized

        ROS_INFO("elastic_bridge: first camera info received.");

        float fx, fy, cx, cy;
        const boost::array<double, 9> & K = msg->K;
        fx = K[0];
        fy = K[4];
        cx = K[2];
        cy = K[5];
        m_height = msg->height;
        m_width = msg->width;
        m_focal_x = fx;
        m_focal_y = fy;
        m_center_x = cx;
        m_center_y = cy;

        cameraInfo_sub.shutdown();
        InitElasticFusion(m_height, m_width, fx, fy, cx, cy);
        m_cameraInfoOK = true;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoPtr& msg) {
        boost::mutex::scoped_lock lock(m_mutex);
        m_camera_info = msg;
        m_cond_var.notify_all();
    }

    void ImagesCallbackWorker(const sensor_msgs::ImageConstPtr& imageColor, const sensor_msgs::ImageConstPtr& imageDepth) {
        if ((m_cameraInfoOK) && (m_started)) {

            //ROS_INFO("elastic_bridge: received frame %d", int(m_frame_count));

            const std::string encoding = imageDepth->encoding;
            const std::string color_encoding = imageColor->encoding;
            const bool bigendian = imageDepth->is_bigendian;

            ros::Time header_stamp = imageColor->header.stamp;

            int n_pixel = m_height * m_width;

            int64_t time = header_stamp.toNSec();
            std::vector<uint16> DEPTH_data(n_pixel);
            std::vector<uint8> RGB_data(n_pixel * 3);

            for (int r = 0; r < n_pixel; r++) {
                if (encoding == "32FC1") {
                    float f;
                    std::memcpy(&f, &(imageDepth->data[4 * r]), sizeof (f));
                    if (std::isfinite(f))
                      DEPTH_data[r] = uint16(f * 1000);
                    else
                      DEPTH_data[r] = 0;
                } else if (encoding == "16UC1") {
                    if (bigendian) {
                        DEPTH_data[r] = (imageDepth->data[2 * r] * 256) + (imageDepth->data[2 * r + 1]);
                    } else {
                        DEPTH_data[r] = (imageDepth->data[2 * r]) + (imageDepth->data[2 * r + 1] * 256);
                    }
                } else {
                    ROS_ERROR("Unknown encoding: %s", encoding.c_str());
                    return;
                }
            }

            for (int i = 0; i < n_pixel; i++) {
                if (color_encoding == "rgb8") {
                    for (int h = 0; h < 3; h++)
                        RGB_data[i * 3 + h] = imageColor->data[i * 3 + (2 - h)];
                } else if (color_encoding == "bgr8") {
                    std::memcpy(&(RGB_data[i * 3]), &(imageColor->data[i * 3]), 3);
                } else {
                    ROS_ERROR("Unknown color encoding: %s", color_encoding.c_str());
                    return;
                }
            }

            if (m_black_to_nan)
            {
                for (int i = 0; i < n_pixel; i++) {
                    if (RGB_data[i * 3 + 0] == 0 &&
                        RGB_data[i * 3 + 1] == 0 &&
                        RGB_data[i * 3 + 2] == 0)
                        DEPTH_data[i] = 0;
                }
            }

            Eigen::Matrix4f currPose;
            if (m_poseFromTFFirst || m_poseFromTFAlways) {
                Eigen::Matrix4f pose = readTF();
                m_eFusion->processFrame(RGB_data.data(), DEPTH_data.data(), time, &pose);
                currPose = pose;
            } else {
                m_eFusion->processFrame(RGB_data.data(), DEPTH_data.data(), time);
                currPose = m_eFusion->getCurrPose();
            }

            imagePublish(currPose, DEPTH_data, RGB_data);

            sendTF(currPose, m_world_frame, m_camera_frame);
            
            if (m_periodic_world_publication) {
                if ((m_frame_count % m_periodic_world_publication) + 1 == m_periodic_world_publication) {
                    ROS_INFO("Publishing GlobalModel (during execution)");
                    publishWorldExec();
                    ROS_INFO("Published GlobalModel (during execution)");
                }
            }

            //ROS_INFO("elastic_bridge: processed frame %d, point count: %d", m_frame_count, m_eFusion->getGlobalModel().lastCount());
            m_frame_count++;
        }
    }

    void ImagesCallback(const sensor_msgs::ImageConstPtr& imageColor, const sensor_msgs::ImageConstPtr& imageDepth) {
        boost::mutex::scoped_lock lock(m_mutex);
        m_image_color = imageColor;
        m_image_depth = imageDepth;
        m_cond_var.notify_all();
    }

    void InitElasticFusion(int Height, int Width, float fx, float fy, float cx, float cy) {
        ROS_INFO("elastic_bridge: Initializing Elastic Fusion...");
        ROS_INFO("elastic_bridge: w %d, h %d, fx %f, fy %f, cx %f, cy %f",
                 Width, Height, fx, fy, cx, cy);

        Resolution::getInstance(Width, Height);
        Intrinsics::getInstance(fx, fy, cx, cy);

        InitFakeOpenGLContext(m_display_name);

        m_guid_counter = 0;

        m_eFusion = new ElasticFusion(200, 35000, 5e-05, 1e-05, !m_poseFromTFAlways);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        ROS_INFO("elastic_bridge: Elastic Fusion initialized.");
    }

    void run() {
        boost::mutex::scoped_lock lock(m_mutex);
        while (!ros::isShuttingDown()) {
            m_cond_var.wait_for(lock, boost::chrono::duration<uint64, boost::milli>(100));

            if (m_image_color && m_image_depth) {
                ImagesCallbackWorker(m_image_color, m_image_depth);
                m_image_color.reset();
                m_image_depth.reset();
            }

            if (m_camera_info) {
                cameraInfoCallbackWorker(m_camera_info);
                m_camera_info.reset();
            }

            if (m_download_request) {
                m_point2 = this->getPC2(&m_guids,&m_luids);
                m_download_ready = true;
                m_cond_var.notify_all();
            }
        }

        m_cond_var.notify_all(); // upon exiting, wake up everyone
    }

    void init() {
        boost::mutex::scoped_lock lock(m_mutex);

        int param_int;
        std::string param_string;
        
        m_nh.param<std::string>("TOPIC_IMAGE_COLOR", m_TopicImageColor, "/camera/rgb/image_rect_color");
        m_nh.param<std::string>("TOPIC_IMAGE_DEPTH", m_TopicImageDepth, "/camera/depth_registered/sw_registered/image_rect");
        m_nh.param<std::string>("TOPIC_CAMERA_INFO", m_TopicCameraInfo, "/camera/rgb/camera_info");
        m_nh.param<std::string>("WORLD_FRAME", m_world_frame, "/first_frame");
        m_nh.param<std::string>("CAMERA_FRAME", m_camera_frame, "/camera_frame");

        m_nh.param<bool>("AUTOSTART", m_started, false);
        m_nh.param<std::string>("DISPLAY_NAME", m_display_name, ""); // empty for auto-detect current
                                                                     // set to Xorg display name otherwise (e.g. ":0")

        m_nh.param<bool>("BLACK_TO_NAN", m_black_to_nan, false);

        m_nh.param<int>("PERIODIC_WORLD_PUBLICATION", param_int, 0); // 0 to disable
        m_periodic_world_publication = std::max<int>(param_int, 0);
        m_nh.param<std::string>("TOPIC_PERIODIC_WORLD_PUB", m_topic_periodic_world_pub, "/elastic_world_pub");
        m_periodic_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>(m_topic_periodic_world_pub, 1);

        m_nh.param<std::string>("TOPIC_FRAME_STATE", param_string, "/elastic_frame_state_stable");
        m_frame_state_stable_pub = m_nh.advertise<elastic_bridge::FrameState>(param_string, 1);

        m_nh.param<std::string>("TOPIC_CURRENT_VIEW", param_string, "/elastic_current_view");
        m_image_pub = m_nh.advertise<sensor_msgs::Image>(param_string, 1);

        #ifdef ELASTIC_BRIDGE_EXTENSION_GUID
          m_nh.param<std::string>("TOPIC_GUID_IMAGE", param_string, "/elastic_guid_image");
          m_guid_image_pub = m_nh.advertise<sensor_msgs::Image>(param_string, 1);
        #endif

        cameraInfo_sub = m_nh.subscribe(m_TopicCameraInfo, 1, &ElasticBridge::cameraInfoCallback, this);

        m_nh.param<bool>("TF_POSE_ALWAYS", m_poseFromTFAlways, false);
        m_nh.param<bool>("TF_POSE_FIRST", m_poseFromTFFirst, false);
        m_nh.param<std::string>("TF_INPUT_WORLD_FRAME", m_input_world_frame, "/world");
        m_nh.param<std::string>("TF_INPUT_CAMERA_FRAME", m_input_camera_frame, "/robot");

        m_nh.param<std::string>("TOPIC_SCAN_READY", param_string, "/elastic_scan_start");
        scanReady_sub = m_nh.subscribe(param_string, 1, &ElasticBridge::scanReadyCallback, this);
        m_nh.param<std::string>("TOPIC_SCAN_FINISH", param_string, "/elastic_scan_end");
        scanFinish_sub = m_nh.subscribe(param_string, 1, &ElasticBridge::scanFinishCallback, this);

        m_imageColor_sub = ImageFilterSubscriberPtr(new ImageFilterSubscriber(m_nh, m_TopicImageColor, 1));
        m_imageDepth_sub = ImageFilterSubscriberPtr(new ImageFilterSubscriber(m_nh, m_TopicImageDepth, 1));

        m_sync_sub = ATSynchronizerPtr(new ATSynchronizer(ATSyncPolicy(10), *m_imageColor_sub, *m_imageDepth_sub));
        m_sync_sub->registerCallback(boost::bind(&ElasticBridge::ImagesCallback, this, _1, _2));
    }

    ElasticBridge(ros::NodeHandle & nh) : m_nh(nh) {
        init();
    }

    ~ElasticBridge() {
        if (m_eFusion)
            delete m_eFusion;
    }

};

class DownloadPC2_Action {
protected:
    ros::NodeHandle & m_nh;
    actionlib::SimpleActionServer<elastic_bridge::downloadPointcloud2Action> m_as;
    ElasticBridge * eb;

public:

    DownloadPC2_Action(const std::string name, ElasticBridge * eb_, ros::NodeHandle & nh) :
      m_nh(nh),
      m_as(m_nh, name, boost::bind(&DownloadPC2_Action::executeCB, this, _1), false) {
        eb = eb_;
        m_as.start();
    }

    void executeCB(const elastic_bridge::downloadPointcloud2GoalConstPtr &goal) {
        if (goal->stop) {
            eb->scanFinishCallback(std_msgs::EmptyConstPtr(new std_msgs::Empty()));
        }

        elastic_bridge::downloadPointcloud2Result result;

        sensor_msgs::PointCloud2ConstPtr pointCloud2 = eb->requestDownload(result.guids, result.luids);
        if (!pointCloud2) {
          m_as.setAborted();
          return;
        }

        result.pointcloud = *pointCloud2;
        m_as.setSucceeded(result);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "elastic_node");

    ros::NodeHandle nh("~");

    ElasticBridge eb(nh);

    std::string param_string;
    nh.param<std::string>("SAVE_PCL_ACTION", param_string, "/save_pcl");
    DownloadPC2_Action action(param_string, &eb, nh);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    eb.run();

    return 0;
}
