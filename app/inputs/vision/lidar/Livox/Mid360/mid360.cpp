#include "mid360.h"

mid360::mid360()
{

}

std::atomic<lyn_step> mid360::step;

uint64_t bytesToTimestampMemcpy(const uint8_t bytes[8]) {
    uint64_t result;
    std::memcpy(&result, bytes, 8);
    return result;
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr) {
        return;
    }
#if 0
    logger_mid360_PointCloudCallback << __FUNCTION__ << "point cloud handle: " << handle << ", " << "data_num: " <<  data->dot_num << ", "
                 << "data_type: " << int(data->data_type) << ", " << "length: " << data->length << ", " << "frame_counter: " << int(data->frame_cnt)
                 << std::endl;
#endif
#if false
    LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
    for (uint32_t i = 0; i < data->dot_num; i++)
    {
        {
            int32_t i_x = p_point_data[i].x;
            int32_t i_y = p_point_data[i].y;
            int32_t i_z = p_point_data[i].z;

            std::vector<int> index_livox_point1;
            index_livox_point1.push_back(i_x);
            index_livox_point1.push_back(i_y);
            index_livox_point1.push_back(i_z);

            oss << " x: " << p_point_data[i].x << ", y: " << p_point_data[i].y << ", z: " << p_point_data[i].z ;
        }
    }
    oss << "\n";
#endif
#if 0
    logger_mid360_PointCloudCallback << oss.str();
#endif
    static LivoxLidarCartesianHighRawPoint* LivoxLidarp_point_data;
    if (data->data_type == kLivoxLidarCartesianCoordinateHighData)
    {
        LivoxLidarp_point_data = (LivoxLidarCartesianHighRawPoint*)data->data;                                                       //(LivoxLidarCartesianHighRawPoint *)
        LivoxLidarp_point_data_mat = cv::Mat(1, 96, CV_8UC(14), LivoxLidarp_point_data);
        std::vector<cv::Mat> LivoxLidarp_point_data_mat_14;
        cv::split(LivoxLidarp_point_data_mat, LivoxLidarp_point_data_mat_14);

        LivoxLidarp_point_data_mat_reflectivity = std::move(LivoxLidarp_point_data_mat_14[12]);
        LivoxLidarp_point_data_mat_tag = std::move(LivoxLidarp_point_data_mat_14[13]);

        cv::Mat LivoxLidarp_point_data_mat_xyz = cv::Mat(1, data->dot_num, CV_32FC3);
        LivoxLidarp_point_data_mat_xyz.setTo(0);
        for (int j = 0; j < data->dot_num; j++)
        {
            LivoxLidarp_point_data_mat_xyz.at<cv::Vec3f>(0, j) = cv::Vec3f(float(LivoxLidarp_point_data[j].x), float(LivoxLidarp_point_data[j].y), float(LivoxLidarp_point_data[j].z));
        }

        lyn_Info info;

        info["LivoxLidarp_point_data_mat_xyz"] = std::move(LivoxLidarp_point_data_mat_xyz);
#if 0
        oss << "LivoxLidarp_point_data_mat_xyz.cols: " << LivoxLidarp_point_data_mat_xyz.cols
            << " rows: " << LivoxLidarp_point_data_mat_xyz.rows << " \n" << LivoxLidarp_point_data_mat_xyz << " \n";

#endif
        info["LivoxLidarp_point_data_mat_reflectivity"] = std::move(LivoxLidarp_point_data_mat_reflectivity);
        info["LivoxLidarp_point_data_mat_tag"] = std::move(LivoxLidarp_point_data_mat_tag);
        logger_mid360_mutex.lock();
        info["LivoxLidarp_Imu_data_mat"] = std::move(LivoxLidarp_Imu_data_mat);
        info["LivoxLidarp_Imu_timestamp"] = LivoxLidarp_Imu_timestamp;
        info["LivoxLidarp_Imu_data_mat_update"] = LivoxLidarp_Imu_data_mat_update;
        LivoxLidarp_Imu_data_mat_update = 0;
        logger_mid360_mutex.unlock();
        info["version"] = data->version;
        info["length"] = int(data->length);
        info["time_interval"] = data->time_interval;
        info["dot_num"] = data->dot_num;
        info["udp_cnt"] = data->udp_cnt;
        info["frame_cnt"] = data->frame_cnt;
        info["data_type"] = data->data_type;
        info["time_type"] = data->time_type;
        info["rsvd"] = data->rsvd[0];
        info["crc32"] = data->crc32;
        info["timestamp"] = bytesToTimestampMemcpy(data->timestamp);

        LivoxLidarp_point_data_mat_infos.push_back(info);
        if(mid360::step.load() == lyn_step_updating)
        {
            LivoxLidarp_point_data_mat_infos_Public = std::move(LivoxLidarp_point_data_mat_infos);
            mid360::step.store(lyn_step_update_complete);
        }
    }
    else if (data->data_type == kLivoxLidarCartesianCoordinateLowData)
    {
        LivoxLidarCartesianLowRawPoint *p_point_data = (LivoxLidarCartesianLowRawPoint *)data->data;
    } else if (data->data_type == kLivoxLidarSphericalCoordinateData)
    {
        LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint *)data->data;
    }
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type,  LivoxLidarEthernetPacket* data, void* client_data) {
    if (data == nullptr)
    {
        return;
    }
    logger_mid360_mutex.lock();
    LivoxLidarp_Imu_data_mat = cv::Mat(1, 2, CV_32FC3, data->data);
    LivoxLidarp_Imu_timestamp = long(bytesToTimestampMemcpy(data->timestamp));
    LivoxLidarp_Imu_data_mat_update = 1;
    logger_mid360_mutex.unlock();
#if 0
    logger_mid360_ImuDataCallback << ""
        << "LivoxLidarp_Imu_data_mat: " << LivoxLidarp_Imu_timestamp << ", " << LivoxLidarp_Imu_data_mat
        << std::endl;
#endif
}

// void OnLidarSetIpCallback(livox_vehicle_status status, uint32_t handle, uint8_t ret_code, void*) {
//   if (status == kVehicleStatusSuccess) {
//     printf("lidar set ip slot: %d, ret_code: %d\n",
//       slot, ret_code);
//   } else if (status == kVehicleStatusTimeout) {
//     printf("lidar set ip number timeout\n");
//   }
// }

void WorkModeCallback(livox_status status, uint32_t handle,LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr)
    {
        return;
    }
    logger_mid360 << __FUNCTION__ << "WorkModeCallack, status: " << status << ", handle: " << handle
                            << ", ret_code: " << response->ret_code << ", error_key: " << response->error_key << "";
}

void RebootCallback(livox_status status, uint32_t handle, LivoxLidarRebootResponse* response, void* client_data) {
    if (response == nullptr)
    {
        return;
    }
    logger_mid360 << __FUNCTION__ << "RebootCallback, status: " << status << ", handle: " << handle << ", ret_code: " << response->ret_code << "";
}

void SetIpInfoCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
    if (response == nullptr)
    {
        return;
    }

    logger_mid360 << __FUNCTION__ <<"LivoxLidarIpInfoCallback, status: " << status << ", handle: " << handle << ", ret_code: "
                                    << response->ret_code << ", error_key: " << response->error_key << "";
    if (response->ret_code == 0 && response->error_key == 0)
    {
        LivoxLidarRequestReboot(handle, RebootCallback, nullptr);
    }
}

void QueryInternalInfoCallback(livox_status status, uint32_t handle,
                               LivoxLidarDiagInternalInfoResponse* response, void* client_data)
{
    if (status != kLivoxLidarStatusSuccess)
    {
        logger_mid360 << __FUNCTION__ << "Query lidar internal info failed." << std::endl;
        QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);
        return;
    }

    if (response == nullptr)
    {
        return;
    }

    uint8_t host_point_ipaddr[4] {0};
    uint16_t host_point_port = 0;
    uint16_t lidar_point_port = 0;

    uint8_t host_imu_ipaddr[4] {0};
    uint16_t host_imu_data_port = 0;
    uint16_t lidar_imu_data_port = 0;

    uint16_t off = 0;
    for (uint8_t i = 0; i < response->param_num; ++i)
    {
        LivoxLidarKeyValueParam* kv = (LivoxLidarKeyValueParam*)&response->data[off];
        if (kv->key == kKeyLidarPointDataHostIpCfg)
        {
            memcpy(host_point_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_point_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_point_port), &(kv->value[6]), sizeof(uint16_t));
        } else if (kv->key == kKeyLidarImuHostIpCfg)
        {
            memcpy(host_imu_ipaddr, &(kv->value[0]), sizeof(uint8_t) * 4);
            memcpy(&(host_imu_data_port), &(kv->value[4]), sizeof(uint16_t));
            memcpy(&(lidar_imu_data_port), &(kv->value[6]), sizeof(uint16_t));
        }
        off += sizeof(uint16_t) * 2;
        off += kv->length;
    }

    logger_mid360 << __FUNCTION__ << "Host point cloud ip addr: " << host_point_ipaddr[0] << ". " << host_point_ipaddr[1]
                 << ". " << host_point_ipaddr[2] << ". " << host_point_ipaddr[3] << ", host point cloud port: "
                 << host_point_port << ", lidar point cloud port: " << lidar_point_port << std::endl;
    logger_mid360 << __FUNCTION__ <<"Host imu ip addr: " << host_imu_ipaddr[0] << ". "
                 << host_imu_ipaddr[1] << ". " << host_imu_ipaddr[2] << ". " << host_imu_ipaddr[3]
                 << ", host imu port: " << host_imu_data_port << ", lidar imu port: " << lidar_imu_data_port << std::endl;
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
{
    if (info == nullptr)
    {
        printf("lidar info change callback failed, the info is nullptr.\n");
        return;
    }
    printf("LidarInfoChangeCallback Lidar handle: %u SN: %s\n", handle, info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, WorkModeCallback, nullptr);                   // set the work mode to kLivoxLidarNormal, namely start the lidar
    QueryLivoxLidarInternalInfo(handle, QueryInternalInfoCallback, nullptr);

    // LivoxLidarIpInfo lidar_ip_info;
    // strcpy(lidar_ip_info.ip_addr, "192.168.1.10");
    // strcpy(lidar_ip_info.net_mask, "255.255.255.0");
    // strcpy(lidar_ip_info.gw_addr, "192.168.1.1");
    // SetLivoxLidarLidarIp(handle, &lidar_ip_info, SetIpInfoCallback, nullptr);
}

void LivoxLidarPushMsgCallback(const uint32_t handle, const uint8_t dev_type, const char* info, void* client_data) {
    struct in_addr tmp_addr;
    tmp_addr.s_addr = handle;
    logger_mid360 << "handle: " << handle << ", ip: " << inet_ntoa(tmp_addr) << ", push msg info: " << std::endl;
    logger_mid360 << info << std::endl;
    return;
}

int mid360::run(int argc, char *argv[])
{
    const std::string path = "/home/lyenbot/src/mid360_config.json";
    logger_mid360_PointCloudCallback.run("logger_mid360_PointCloudCallback.log", false);
    logger_mid360_ImuDataCallback.run("logger_mid360_ImuDataCallback.log", false);
    logger_mid360.run("logger_mid360.log");

    if (!LivoxLidarSdkInit(path.c_str()))
    {                                                                                              // REQUIRED, to init Livox SDK2
        printf("Livox Init Failed\n");
        LivoxLidarSdkUninit();
        return -1;
    }
    DisableLivoxSdkConsoleLogger();
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);                                  // REQUIRED, to get point cloud data via 'PointCloudCallback'
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);                                        // OPTIONAL, to get imu data via 'ImuDataCallback' some lidar types DO NOT contain an imu component
    SetLivoxLidarInfoCallback(LivoxLidarPushMsgCallback, nullptr);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);                             // REQUIRED, to get a handle to targeted lidar and set its work mode to NORMAL

    return 0;
}

int mid360::work(lyn_info &info)
{
    if(info.step["LivoxLidarp_point_data_mat_infos"] == lyn_step_request)
    {
        info.step["LivoxLidarp_point_data_mat_infos"] = lyn_step_updating;
        step.store(lyn_step_updating);
    }

    if(step.load() == lyn_step_update_complete)
    {
        info.step["LivoxLidarp_point_data_mat_infos"] = lyn_step_update_complete;
        info.infos["LivoxLidarp_point_data_mat_infos"] = std::move(LivoxLidarp_point_data_mat_infos_Public);
        step.store(lyn_step_copying);
    }
    return 0;
}

mid360::~mid360()
{
    LivoxLidarSdkUninit();
}
