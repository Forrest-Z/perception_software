#pragma once

#include "cyber/cyber.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <iostream>

namespace drivers {
namespace camera {


class TimeTsServices {
    public:
        static TimeTsServices* GetInstance();
        static void ReleaseTimeTsServices();

        void StartThread();
        void StopThread();
        void GetSyncInfo(uint32_t* sec, uint32_t* nsec);
        void SyncTs();
        // bool GetGatewaySyncInfo(TsInfo* p_ts_info, const std::string yaml_path);

    private:
        static TimeTsServices* instance_;
        TimeTsServices& operator=(TimeTsServices&);

        uint32_t sync_sec_ = 0;
        uint32_t sync_nsec_ = 0;
        bool thread_is_run_ = false;
        uint64_t system_timestamp = 0;

        std::thread thread_;
        
        std::mutex base_mutex_;
};
}  // namespace camera
}  // namespace drivers