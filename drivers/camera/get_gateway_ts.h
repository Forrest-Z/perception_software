#pragma once

#include "cyber/cyber.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <iostream>

namespace drivers {
namespace camera {

class GetGatewayTs {
    public:
        static GetGatewayTs* GetInstance();
        static void ReleaseGateWay();

        void StartThread();
        void StopThread();
        bool SetUdpConnection();
        void CloseConnection();
        void GetSyncInfo(const std::string& frame_id, uint32_t* sec, uint32_t* nsec);
        void SyncTs();
        // bool GetGatewaySyncInfo(TsInfo* p_ts_info, const std::string yaml_path);

    private:
        static GetGatewayTs* instance_;
        GetGatewayTs& operator=(GetGatewayTs&);
        std::string ip_ = "192.168.1.102";
        uint32_t port_ = 5500;
        int32_t sock_;
        struct sockaddr_in recv_addr_;
        uint32_t sync_sec_ = 0;
        uint32_t sync_nsec_ = 0;
        bool thread_is_run_ = false;
        uint64_t system_timestamp = 0;

        std::thread udp_thread_;
        static std::mutex base_mutex_;

        std::map<std::string, uint64_t> last_map;
};
}  // namespace camera
}  // namespace drivers