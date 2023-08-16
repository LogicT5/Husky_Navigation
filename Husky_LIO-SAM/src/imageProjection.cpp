/****************************************************************************
*imageProjection*

功能：
+ imageProjecttion的主要功能是订阅原始点云数据和imu数据，根据高频的imu信息对点云成像时雷达的位移和旋转造成的畸变进行校正
+ 同时，在发布去畸变点云的时候加入IMU输出的角度和IMU里程计（imuPreintegration）的角度和位姿作为该帧的初始位姿，作为图优化的初始估计
+ 并且，要对点云的Range进行计算，同时记录每个点的行列，以便在特征提取中被使用

订阅：
1. IMU原始数据
2. Lidar原始数据
3. IMU里程计，来自imuPreintegration. IMU里程计融合了低频激光里程计数据（更准确）和高频的IMU数据（噪声较大），比直接用原始IMU数据做积分更准确

发布：
1. 去畸变后的点云及附属信息，
包括：原始点云、
           去畸变点云、
           该帧点云的初始旋转旋转角（来自IMU原始roll、pitch、yaw）
           该帧点云的初始位姿（来自IMU里程计）

流程：
1. 接收到一帧点云
2. 从IMU原始数据队列找到该帧点云时间戳对应的数据，将IMU的roll、pitch、yaw塞进准备发布的该帧点云信息
3. 提取该帧点云的起止时间戳（激光雷达点云的每个点都有相对于该帧起始时间的时间间隔）
4. 对起止时间内的IMU数据进行角度积分，得到该帧点云每个时刻对应的旋转。
（注意，这里算法使用的是简单的角度累加，实际上是积分的近似，但是在很短的时间内，10Hz雷达对应100ms的扫描时间，近似的累加可以代替角度积分。
猜想这里是因为点云去畸变是整个SLAM流程的入口，要保证足够的实时性，因此用累加代替真正的角度积分）
5. 遍历该帧点云每个点，旋转到起始点坐标系
6. 从IMU里程计提取该帧点云对应的位姿（包括位置和旋转），塞进准备发布的该帧点云信息
7. 发布该帧点云信息

修改备注：


*******************************************************************************/
#include "utility.h"
#include "lio_sam/cloud_info.h"

// ###############雷达数据类型 START########################
/* Velodyne雷达的数据类型：
 * 这里用了16byts对齐
 */
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct HesaiPointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

/* Ouster雷达数据类型：
 */
struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(uint16_t, reflectivity, reflectivity)(uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

// mulran datasets
struct MulranPointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(MulranPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint32_t, t, t)(int, ring, ring))
// ###############雷达数据类型  END########################

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

/*IMU数据队列长度*/
const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:
    // IMU队列和IMU odom队列线程锁
    std::mutex imuLock;
    std::mutex odoLock;

    // 原始雷达点云数据的订阅
    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    // 发布矫正后的点云，有效点
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    // IMU数据的订阅
    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue; // IMU数据队列

    // IMU里程计数据的订阅
    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue; // IMU里程计数据队列

    // 点云数据
    std::deque<sensor_msgs::PointCloud2> cloudQueue; // 点云数据的队列
    sensor_msgs::PointCloud2 currentCloudMsg;        // 从点云队列中提取出当前点云帧做处理

    // 记录每一帧点云从起始到结束过程所有的IMU数据，imuRotX,Y,Z是对这一段时间内的角速度累加的结果
    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;   // 记录每一帧点云起止过程中imuTime、imuRotXYZ的实际数据长度
    bool firstPointFlag; // 第一点标记 - 处理第一个点时   *将该点的旋转取逆，记录到transStartInverse中，后续方便计算旋转的增量
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;           // 当前帧原始激光点云
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn; // Ouster雷达数据类型：
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn; // Mulran数据类型：
    pcl::PointCloud<PointType>::Ptr fullCloud;                // 从fullCloud中提取有效点
                                                              // if(featureExtracted)
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    int deskewFlag;   // 当点云的time/t字段不可用，也就是点云中不包含每个点的时间戳，无法进行去畸变，直接返回原始点云
                      // if(featureExtracted)
    cv::Mat rangeMat; // 存储点云的range图像

    bool odomDeskewFlag; // 是否有合适的IMU里程计数据
    // 当前激光帧起止时刻对应imu里程计位姿变换，该变换对应的平移增量；
    // 用于插值计算当前激光帧起止时间范围内，每一时刻的位置
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo; // 点云信息：雷达线数
    double timeScanCur;            // 当前雷达帧的起始时间
    double timeScanEnd;            // 当前雷达帧的结束时间
    std_msgs::Header cloudHeader;  // 当前雷达帧的Header

    vector<int> columnIdnCountVec; // ？？？

public:
    ImageProjection() : deskewFlag(0)
    {
        // 订阅Topic
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // 这里的odom是由imuPreintegration积分计算得到的每时刻imu位姿
        subOdom = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布Topic
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>("husky_lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("husky_lio_sam/deskew/cloud_info", 1);

        // 初始化
        allocateMemory();
        // 重置参数
        resetParameters();

        // 日志
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    /*初始化*/
    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        if (featureExtracted)
        {
            extractedCloud.reset(new pcl::PointCloud<PointType>());

            fullCloud->points.resize(N_SCAN * Horizon_SCAN);

            cloudInfo.startRingIndex.assign(N_SCAN, 0);
            cloudInfo.endRingIndex.assign(N_SCAN, 0);

            cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
            cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);
        }

        resetParameters();
    }

    /*重置参数，接收到一帧点云重置一次*/
    void resetParameters()
    {
        laserCloudIn->clear();
        if (featureExtracted)
        {
            extractedCloud->clear();
            // reset range matrix for range image projection
            rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        }

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        if (featureExtracted)
            columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection() {}

    /* IMU订阅的回调函数 */
    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg)
    {
        // imu原始测量数据转换到lidar系，加速度、角速度、RPY
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        // 上锁，IMU数据队列添加数据的时候不可用
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // std::cout << "\033[1;31m" << "[ H-LIO-SAM DEBUG ] " << "\033[1;33m" << " IMU data" << std::endl;
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    /* 订阅imu里程计，由imuPreintegration积分计算得到的每时刻imu位姿 */
    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg); // IMU队列加入IMU里程计位姿
    }

    /*点云订阅回调函数
     *
     */
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // 添加点云数据，提取最早帧
        if (!cachePointCloud(laserCloudMsg))
            {
                ///*Debug
                #ifdef DEBUG
                if(debug){
                std::cout << "\033[1;31m" << "[ H-LIO-SAM DEBUG ] " << "\033[1;33m" << "ImageProjection-cloudHandler" << std::endl;
                std::cout << "\033[1;35m" << " cachePointCloud(laserCloudMsg)  return :  "<< "\033[1;36m"<< std::endl;
                std::cout << "\033[0m" << std::endl;}
                #endif//*/
                return;
            }

        if (!deskewInfo())
            {
                ///*Debug
                #ifdef DEBUG
                if(debug){
                std::cout << "\033[1;31m" << "[ H-LIO-SAM DEBUG ] " << "\033[1;33m" << "ImageProjection-cloudHandler" << std::endl;
                std::cout << "\033[1;35m" << " deskewInfo()  return :  "<< "\033[1;36m"<< std::endl;
                std::cout << "\033[0m" << std::endl;}
                #endif//*/
                return;
            }

        projectPointCloud();

        if (featureExtracted)
            cloudExtraction();

        publishClouds();

        resetParameters();
    }

    /* cachePointCloud
     *   添加一帧激光点云到队列，取出最早的一帧作为当前帧，计算起始时间，检查数据有效性
     *   param(out):    currentCloudMsg     // 当前帧
     *                          cloudHeader            // 当前帧头部
     *                          timeScanCur             //当前帧起始时刻
     *                          timeScanEnd             //当前帧结束时刻
     */
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            // 至少两帧在队列里才能处理，1，当前帧  2，最新帧
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front()); // 最早帧作为当前帧
        cloudQueue.pop_front();                          // pop front 帧，之后第二帧变为front
        // 区分雷达类型
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            // 点云格式转为pcl
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn); // moveFromeROSMsg是浅拷贝，使源数据作废
        }
        else if (sensor == SensorType::HESAI)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = 1;
                int scanID = 0;
                float angle = atan(dst.z / sqrt(dst.x * dst.x + dst.y * dst.y)) * 180 / M_PI; //-90�ȵ�90��
                if (angle > 2)                                                                // 1~5
                {
                    if (angle > 2 && angle <= 3.5)
                        scanID = 5;
                    else if (angle > 3.5 && angle <= 5.5)
                        scanID = 4;
                    else if (angle > 5.5 && angle <= 8.5)
                        scanID = 3;
                    else if (angle > 8.5 && angle <= 11.5)
                        scanID = 2;
                    else if (angle > 11.5 && angle <= 16)
                        scanID = 1;
                }
                else if (angle < -6)
                {
                    if (angle < -6 && angle >= -14)
                        scanID = 31 + abs(int((angle + 7) / 1 - 0.5));
                    else if (angle < -14 && angle >= -19.5)
                        scanID = 39;
                    else if (angle < -19.5 && angle >= -26)
                        scanID = 40;
                }
                else if (angle >= -6 && angle <= 2)
                {
                    scanID = 6 + abs(int((angle - 2) / 0.33 - 0.5));
                }
                dst.ring = scanID;
                dst.time = 0;
            }
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::MULRAN)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        } // <!-- liorf_yjz_lucky_boy -->
        else if (sensor == SensorType::ROBOSENSE)
        {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++)
            {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }
        ///*Debug
        #ifdef DEBUG
        if(debug){
        std::cout << "\033[1;31m" << "[ H-LIO-SAM DEBUG ] " << "\033[1;33m" << "ImageProjection-cachePointCloud" << std::endl;
        std::cout << "\033[1;35m" << "SensorType:          "        << "\033[1;36m" << int(sensor)        << std::endl;
        std::cout << "\033[1;35m" << "laserCloudIn size: "        << "\033[1;36m" << laserCloudIn->points.size()   << std::endl;
        std::cout << "\033[0m";}
        #endif //*/

        // get timestamp
        cloudHeader = currentCloudMsg.header;                         // 当前帧头部
        timeScanCur = cloudHeader.stamp.toSec();                      // 当前帧起始时刻
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time; // 当前帧结束时刻
        // 注：点云中激光点的time记录相对于当前帧第一个激光点的时差，第一个点time=0
        #ifdef DEBUG
        if(debug){
        std::cout << "\033[1;35m" << "timeScanCur:  "        << "\033[1;36m" << timeScanCur        << std::endl;
        std::cout << "\033[1;35m" << "timeScanEnd:  "        << "\033[1;36m" << timeScanEnd   << std::endl;
        std::cout << "\033[0m" << std::endl;}
        #endif//*/

        // check dense flag 是否存在无效点
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel 是否存ring通道  static只检查一次
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
                else if (sensor == SensorType::HESAI)
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // check point time 是否存在time通道
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    /*deskeInfo
     *   当前帧起止时刻对应的IMU数据，IMU odom数据
     *
     */
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan 要求imu数据包含激光数据，否则不往下处理了
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        // 当前帧对应的imu数据处理
        imuDeskewInfo();

        // 当前帧对应的imu odom数据处理
        odomDeskewInfo();

        return true;
    }

    /*imuDeskeInfo
     *   当前帧对应的imu数据处理
     *   1、遍历当前激光帧起止时刻（前后扩展0.01s）之间的imu数据，
     *         初始时刻对应imu的姿态角RPY设为当前帧的初始姿态角
     *   2、用角速度、时间积分，计算每一时刻相对于初始时刻的旋转量，初始时刻旋转设为0
     *   注：imu数据都已经转换到lidar系下了
     */
    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        // 从imu队列中删除当前激光帧0.01s以前时刻的imu数据（此时已经无用的imu数据）
        // 向前扩展0.01s
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        // 遍历雷达当前帧起止时刻之间（前后扩展了0.01s）的imu数据
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            if (imuType)
            {
                // (9轴)提取imu姿态角RPY，作为当前lidar帧初始姿态角
                if (currentImuTime <= timeScanCur)
                    // timeScanCur-0.01 到 timeScanCur之间离timecanCur最近一帧数据
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            }

            // 向后扩展0.01s
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0)
            {
                // imu第一帧旋转角初始化
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // 从timeScanCur-0.01 到 timeScanEnd+0.01之间的第二帧开始
            //  get angular velocity 提取imu角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // imu帧间时差
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            // 当前时刻旋转角 = 前一时刻旋转角 + 角速度 * 时差
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        } // end imuQueue

        --imuPointerCur;

        if (imuPointerCur <= 0)
            // 没有合规的imu数据
            return;

        cloudInfo.imuAvailable = true;
    }

    /*odomDeskewInfo
     *   当前帧对应的imu odom数据处理
     *   1、遍历当前激光帧起止时刻（前后扩展0.01s）之间的imu odom数据，
     *         初始时刻对应imu odom
     *   2、用
     *   注：imu数据都已经转换到lidar系下了
     */
    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        // 删除imu odom队列中当前帧起始时刻0.01s之前的时刻的imu odom数据
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        // 必须要有当前帧起始时刻之前的数据
        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // 提取当前帧起始时刻的imu odom
        nav_msgs::Odometry startOdomMsg;

        // 提取当前帧起始时刻的imu odom
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        // 提取imu odom 姿态
        tf::Quaternion orientation; // imu odom 姿态（四元数）
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw; // imu odom 欧拉角
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 用当前帧起始时刻的imu里程计，初始化lidar位姿，后面用于mapOptmization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw = yaw;

        cloudInfo.odomAvailable = true;

        // 在扫描结束时获取最终的里程计数据
        odomDeskewFlag = false;

        // 如果当前帧结束时刻之后没有imu odom数据，返回
        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        // 提取当前帧结束时刻的imu odom数据
        nav_msgs::Odometry endOdomMsg;

        // 提取当前帧结束时刻的imu odom
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        // 如果起止时刻的imu odom的方差不等，返回
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // 起止时刻imu odom的相对变换
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // 从相对变换，提取增量平移、旋转（欧拉角）
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    /**
     * 在当前激光帧起止时间范围内，计算某一时刻的旋转（相对于起始时刻的旋转增量）
     * input:   double pointTime
     *              float *rotXCur
     *              float *rotYCur
     *              float *rotZCur
     * param(out):
     */
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    /**
     * 在当前激光帧起止时间范围内，计算某一时刻的旋转（相对于起始时刻的旋转增量）
     */
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        // 如果传感器移动速度较慢，例如人行走的速度，那么可以认为激光在一帧时间范围内，
        // 平移量小到可以忽略不计
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow,
        // like walking speed, positional deskew seems to have little benefits.
        // Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    /**deskewPoint
     * input:     PointType *point
     * output:   坐标变换后的点
     * 激光运动畸变校正
     * 利用当前帧起止时刻之间的imu数据计算旋转增量，
     * imu odom数据计算平移增量，
     * 进而将每一时刻激光点位置变换到第一个激光点坐标系下，进行运动补偿
     */
    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        // relTime是当前激光点相对于所在当前帧起始时刻的时间，
        // pointTime则是当前激光点的时间戳
        double pointTime = timeScanCur + relTime;

        // 在当前激光帧起止时间范围内，计算某一时刻的旋转（相对于起始时刻的旋转增量）
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        // 在当前激光帧起止时间范围内，计算某一时刻的平移（相对于起始时刻的平移增量）
        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        // 第一个点的位姿增量（0），求逆
        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start // 当前时刻激光点与第一个激光点的位姿变换
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        // 当前激光点在第一个激光点坐标系下的坐标
        PointType newPoint;
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    /**
     * 当前帧激光点云运动畸变校正
     * 1、检查激光点距离、扫描线是否合规
     * 2、激光运动畸变校正，保存激光点
     */
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();

        // 遍历当前帧激光点云
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // 距离检查
            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            // 雷达激光点扫描线检查
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 扫描线如果有降采样，跳过采样的扫描线这里要跳过
            if (rowIdn % downsampleRate != 0)
                continue;

            if (featureExtracted)
            {
               if(sensor == SensorType::HESAI)
                {
                    // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
                    // fullCloud->push_back(thisPoint);
                    int columnIdn = -1;
                    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                    static float ang_res_x = 360.0 / float(Horizon_SCAN);
                    columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
                    if (columnIdn >= Horizon_SCAN)
                        columnIdn -= Horizon_SCAN;
                    thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

                    // 矩阵存激光点的距离
                    rangeMat.at<float>(rowIdn, columnIdn) = range;

                    // 转换成一维索引，存校正之后的激光点
                    int index = columnIdn + rowIdn * Horizon_SCAN;
                    fullCloud->points[index] = thisPoint;
                }
                else
                {
                    // 水平扫描步长
                    int columnIdn = -1;
                    if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
                    {
                        float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                        static float ang_res_x = 360.0 / float(Horizon_SCAN);
                        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
                        if (columnIdn >= Horizon_SCAN)
                            columnIdn -= Horizon_SCAN;
                    }
                    else if (sensor == SensorType::LIVOX)
                    {
                        columnIdn = columnIdnCountVec[rowIdn];
                        columnIdnCountVec[rowIdn] += 1;
                    }

                    if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                        continue;

                    // 已经存过该点，不再处理
                    if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                        continue;

                    // 激光运动畸变校正
                    // 利用当前帧起止时刻之间的imu数据计算旋转增量，
                    // imu里程计数据计算平移增量，
                    // 进而将每一时刻激光点位置变换到第一个激光点坐标系下，进行运动补偿
                    thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

                    // 矩阵存激光点的距离
                    rangeMat.at<float>(rowIdn, columnIdn) = range;

                    // 转换成一维索引，存校正之后的激光点
                    int index = columnIdn + rowIdn * Horizon_SCAN;
                    fullCloud->points[index] = thisPoint;
                }
            }
            else
            {
                if (i % 3 != 0) // if point_filter_num = 3
                    continue;
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
                fullCloud->push_back(thisPoint);
            }
        }
        ///*Debug
        #ifdef DEBUG
        if(debug){
        std::cout << "\033[1;31m" << "[ H-LIO-SAM DEBUG ] " << "\033[1;33m" << "ImageProjection-projectPointCloud" << std::endl;
        std::cout << "\033[1;35m" << "fullCloud size: "        << "\033[1;36m" <<  fullCloud->points.size()   << std::endl;
        std::cout << "\033[0m" << std::endl;}
        #endif//*/
    }

    /**
     * 提取有效激光点，存extractedCloud
     */
    void cloudExtraction()
    {
        int count = 0; // 有效激光点数量
        // extract segmented cloud for lidar odometry
        // 遍历所有激光点，提取分段云用于激光雷达odom
        for (int i = 0; i < N_SCAN; ++i) // 遍历激光线
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            // 遍历Horizon_SCAN方向
            for (int j = 0; j < Horizon_SCAN; ++j)
            {   
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // 有效激光点
                    // 记录激光点对应的Horizon_SCAN方向上的索引
                    cloudInfo.pointColInd[count] = j;
                    // 激光点距离
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // 加入有效激光点
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            // 记录每根扫描线倒数第5个激光点在一维数组中的索引
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    /**
     * 发布当前帧校正后点云，有效点
     */
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        if (featureExtracted)
        {
            cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        }
        else
        {
            cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        }
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "H_lio_sam");

    ImageProjection IP;

    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
