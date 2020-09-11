#include "get_gateway_ts.h"
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/prctl.h>

namespace drivers {
namespace camera {

#define DATA_SIZE  12
//#define TIMESTEMP_DEBUG
GetGatewayTs* GetGatewayTs::instance_ = NULL;
std::mutex GetGatewayTs::base_mutex_;

GetGatewayTs* GetGatewayTs::GetInstance() {
    if(instance_ == NULL) {
        instance_ = new GetGatewayTs();
        instance_->StartThread();
    }
    return instance_;
}

void GetGatewayTs::ReleaseGateWay() {
    if(instance_ != NULL) {
        instance_->CloseConnection();
        instance_->StopThread();
        delete instance_;
        instance_ = NULL;
        std::cout << "Destroy Singleton Object. " << std::endl;
    }
}

void GetGatewayTs::StopThread() {
    thread_is_run_ = false;
    if (udp_thread_.joinable()) {
        udp_thread_.join();
    }
}

void GetGatewayTs::StartThread() {
    thread_is_run_ = true;
    udp_thread_ = std::thread(&GetGatewayTs::SyncTs, this);
}

bool GetGatewayTs::SetUdpConnection() {
    if((sock_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cout << " Could not build socket " <<std::endl;
        return false;
    }

    bzero(&recv_addr_, sizeof(recv_addr_));

    recv_addr_.sin_family = AF_INET;
    recv_addr_.sin_port = htons(static_cast<uint16_t>(port_));
    recv_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
    int ret = bind(sock_, (struct sockaddr*)&recv_addr_, sizeof(recv_addr_));
    if( ret == -1 ) {
        AERROR << " Could not bind. " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void GetGatewayTs::CloseConnection() {
    std::cout << " Close udp connection ---> " << std::endl;
    close(sock_);
}

void GetGatewayTs::SyncTs() {
    //added thread name by hezb
    //prctl(PR_SET_NAME, std::string("CameraSync").c_str());
    prctl(PR_SET_NAME, "cameraSync");
    //added end

    unsigned int slen = sizeof(recv_addr_);
    uint32_t time_sec, time_nsec, pps_counter;
    std::cout << "---SyncTs enter---" << std::endl;
    
    if(!SetUdpConnection()){
        AERROR << "Set udp connection failed!";
        return;
    }
    
    while (thread_is_run_ && !apollo::cyber::IsShutdown()) {
        uint8_t data_recv[DATA_SIZE];
        memset(data_recv,0,DATA_SIZE);
        auto n_bytes = recvfrom(sock_, data_recv, DATA_SIZE, 0, (struct sockaddr*) &recv_addr_, &slen);
        if(n_bytes < 0) {
            AERROR << "could not receive the data";
            close(sock_);
            break;
        } 

        if(n_bytes == 0) {
            AERROR << "server closed";
            close(sock_);
            break;
        }

        if(n_bytes == DATA_SIZE) {
            std::lock_guard<std::mutex> lock(base_mutex_);
            struct timeval tv;
            gettimeofday(&tv, nullptr);
            system_timestamp = static_cast<uint64_t>(tv.tv_sec) * 1000000000UL + 1000 * tv.tv_usec;
            memcpy(&time_sec, &data_recv[0], 4);
            memcpy(&time_nsec, &data_recv[4], 4);
            memcpy(&pps_counter, &data_recv[8], 4);

            pps_counter = htonl(pps_counter);
            sync_sec_ = htonl(time_sec);
            sync_nsec_ = htonl(time_nsec);
#ifdef TIMESTEMP_DEBUG
            std::cout << "pps counter: " << pps_counter << "\n"
                      << "time sec:  "   << sync_sec_ << "\n"
                      << "time nsec: "   << sync_nsec_ << "." << std::endl;
#endif
        } 
        else
        {
            AERROR << "data len err ,n_bytes" << n_bytes << std::endl;
        }
   }
   close(sock_);
   std::cout << "---SyncTs exit---" << std::endl;
}

void GetGatewayTs::GetSyncInfo(const std::string& frame_id, uint32_t* sec, uint32_t* nsec) {
    std::lock_guard<std::mutex> lock(base_mutex_);
    uint64_t diff = 0;
    if(sync_sec_ != 0) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        uint64_t cur_system_timestamp  = static_cast<uint64_t>(tv.tv_sec) * 1000000000UL + 1000 * tv.tv_usec;
        diff = cur_system_timestamp - system_timestamp;
        if(diff > 1000000000UL) {//solve bug:WSMB-268, timestamp error
            AERROR << "[dpp], diff:" << diff << ", frame_id:" << frame_id << ", cur:" << cur_system_timestamp << ", sys:" << system_timestamp;
            uint64_t last_timestamp = 0;
            try {
                last_timestamp = last_map.at(frame_id);
                diff = last_timestamp + 40000000 - system_timestamp;
            } catch (std::out_of_range e) {
                last_timestamp = cur_system_timestamp;
            }

            AERROR << "[dpp], after calibrate, diff:" << diff << ", frame_id:" << frame_id << ", last:" << last_timestamp;
        }

        last_map[frame_id] = cur_system_timestamp;
    }
    *sec = sync_sec_ + static_cast<uint32_t>(diff / 1000000000UL);
    *nsec = sync_nsec_ + static_cast<uint32_t>(diff % 1000000000UL);
}

}  // namespace camera
}  // namespace drivers
