#include "time_ts_service.h"
#include <sys/time.h>

#include <sys/ioctl.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <linux/rtc.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/time.h>  
#include <unistd.h> 

#include <string.h>
#include <pthread.h>
#include <inttypes.h>

#include <string>
#include <iostream>

#include <termios.h>
#include <mutex>

using namespace std;

namespace drivers {
namespace camera {

#define DATA_SIZE  12

TimeTsServices* TimeTsServices::instance_ = NULL;

typedef struct send_pack{
    uint8_t channel;
    uint8_t data;
}SEND_PACK __attribute__ ((aligned (1)));

#define CHANNEL_LENGTH 10
#define TIME_BUFF_LEN 80

#define TTY_PATH "/dev/ttyS0"
#define IO_PATH "/dev/platformio_timersync"
#define TIME_TS_DEBUG 0

class IOData{
public:
    int fdopen(const char * devicePath)   {
        this->fd = open(devicePath, O_RDWR);

        return true;
    }

    void fdclose()  {
        close(this->fd);
    }

    int getIOData(SEND_PACK * data,int channel)  {
        memset(data,0,sizeof(SEND_PACK)*channel);
        read(this->fd, data, sizeof(SEND_PACK)* channel);

        return true;
    }

    int* getFD()  {
        return &this->fd;
    }
private:
    int fd;
};

typedef struct _st_uart_attr {
    int valid;
    int fd;
    int baud;
    int databit;
    int stopbit;
    int parity;
    int flowct;
    int workmode;
    int interval;
}ST_UART;

class SerialIO
{
public:

    int init_com_dev(const char *com_dev,int none_block,ST_UART *uart_attr)  {
        int fd = -1;
        
        if(none_block)
            fd = open(com_dev, O_RDWR|O_EXCL|O_NOCTTY|O_NONBLOCK);
        else
            fd = open(com_dev, O_RDWR|O_EXCL|O_NOCTTY);
        if (fd < 0) {
            printf("Open Comdev '%s' Error!\n",com_dev);
            return -1;
        }
        
        if(setport(fd,uart_attr->baud,uart_attr->databit,uart_attr->stopbit,uart_attr->parity) != 0) {
            printf("setport error!\n");
            return  -1;
        }
        uart_attr->fd = fd;
        return fd;
    }

    int setport(int fd, int baud, int databits, int stopbits, int parity)  {
        int    baudrate;
        struct termios newtio;
        
        switch (baud)  {
            case 300:
                baudrate = B300;
                break;
            case 600:
                baudrate = B600;
                break;
            case 1200:
                baudrate = B1200;
                break;
            case 2400:
                baudrate = B2400;
                break;
            case 4800:
                baudrate = B4800;
                break;
            case 9600:
                baudrate = B9600;
                break;
            case 19200:
                baudrate = B19200;
                break;
            case 38400:
                baudrate = B38400;
                break;
            case 57600:
                baudrate = B57600;
                break;
            case 115200:
                baudrate = B115200;
                break;
            default :
                return -1;
                break;
        }
        
        tcgetattr(fd, &newtio);
        bzero(&newtio, sizeof(newtio));
        
        //must be sed firstly!
        newtio.c_cflag |= (CLOCAL | CREAD);
        newtio.c_cflag &= ~CSIZE;
        
        switch (databits)  { 
            case 7:
                newtio.c_cflag |= CS7;
                break;
            case 8:
                newtio.c_cflag |= CS8;
                break;
            default:
                return -1;
                break;
        }
        
        switch (parity)  {
            case 0:
            case 'n':
            case 'N':
                newtio.c_cflag &= ~PARENB;
                newtio.c_iflag &= ~INPCK;
                break;
            case 1:
            case 'o':
            case 'O':
                newtio.c_cflag |= (PARODD | PARENB);
                newtio.c_iflag |= INPCK;
                break;
            case 2:
            case 'e':
            case 'E':
                newtio.c_cflag |= PARENB;  
                newtio.c_cflag &= ~PARODD;
                newtio.c_iflag |= INPCK;
                break;
            case 'S':
            case 's':
                newtio.c_cflag &= ~PARENB;
                newtio.c_cflag &= ~CSTOPB;
                break;
            default:
                return -1;
                break;
        } 
        
        switch (stopbits) {   
            case 1:
                newtio.c_cflag &= ~CSTOPB;
                break;
            case 2:
                newtio.c_cflag |= CSTOPB;
                break;
            default:
                return -1;
                break;
        }         
                
        newtio.c_cc[VTIME] = 0;
        newtio.c_cc[VMIN] = 1;
    #if 1
        newtio.c_oflag &= ~OPOST;
        newtio.c_oflag &= ~(ONLCR | OCRNL);
        newtio.c_iflag &= ~(ICRNL | INLCR);
        newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    #endif
        newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //raw mode
        cfsetispeed(&newtio, baudrate);
        cfsetospeed(&newtio, baudrate);
        tcflush(fd, TCIFLUSH); 
        
        if (tcsetattr(fd,TCSANOW, &newtio) != 0) { 
            printf("tcsetattr-error ,%d-%s\n",errno,strerror(errno));
            return -1;
        }  
        
        return 0;
    }

    int openUART(const char*devPath)  {
        memset(&this->st,0,sizeof(ST_UART));
        this->st.interval = 0;
        this->st.baud = 115200;
        this->st.databit = 8;
        this->st.stopbit = 1;
        this->st.parity = 0;
        this->st.workmode = 0;
        this->st.parity = 'n';
        this->fd = init_com_dev(devPath,0,&this->st);
        return 0;
    }

    void closeUART() {
        close(this->fd);
    }


    ssize_t recvUART(char *rcv_buf,int data_len)  {
        memset(rcv_buf,0,data_len);
        return read(this->fd,rcv_buf,data_len-1);
    }
private:
    ST_UART st;
    int fd = 0;

};

class UartTime {
private:
    time_t str_to_time(string buf)  {
        tm tm_;  
        time_t t_,t;
        char buft[128] = {0};
        string time_buf;
        tm* local;
        
        t = time(NULL);
        local = localtime(&t);

        strftime(buft, 64, "%Y-%m-%d ", local);
        time_buf = string(buft)+buf;

        
        strptime(time_buf.c_str(), "%Y-%m-%d %H%M%S", &tm_);
        tm_.tm_isdst = -1;
        t_  = mktime(&tm_);

        return t_;
    }

    time_t get_uart_time()   {
        char rcv_buf[TIME_BUFF_LEN];
        memset(rcv_buf,0,sizeof(char)*TIME_BUFF_LEN);

        ssize_t recv_t = 0;
        time_t u_time_t;

        if ( this->is_sync ) {
            recv_t = sio.recvUART(&rcv_buf[0],TIME_BUFF_LEN);
            while( recv_t > 4) {
                recv_t = sio.recvUART(&rcv_buf[0],TIME_BUFF_LEN);

                dbuffer = dbuffer + string(rcv_buf);
            }
            if (dbuffer.at(0) == '$') {
                string timestr = "";
                dbuffer.erase(0,7);
                ssize_t npos = dbuffer.find_first_of(",");
                timestr = dbuffer.substr(0,npos);

                dbuffer.erase(0,9);
                
                npos = dbuffer.find_first_of("$");
                dbuffer.erase(0,npos);

                //cout<<"return buffer:"<<buffer<<"   "<<npos<<endl;
                u_time_t =  str_to_time(timestr);
                //fprintf(stderr, "%s, %llu\n",timestr.c_str(), u_time_t);
            }

        }else {
           recv_t = sio.recvUART(&rcv_buf[0],TIME_BUFF_LEN);

            dbuffer = dbuffer + string(rcv_buf);
            //cout<<buffer<<endl;
            if (dbuffer.length() >= 80) {
                if (dbuffer.at(0) == '$') {
                    string timestr = "";
                    dbuffer.erase(0,7);
                    ssize_t npos = dbuffer.find_first_of(",");
                    timestr = dbuffer.substr(0,npos);

                    dbuffer.erase(0,9);
                    
                    npos = dbuffer.find_first_of("$");
                    dbuffer.erase(0,npos);

                    //cout<<"return buffer:"<<buffer<<"   "<<npos<<endl;
                    u_time_t =  str_to_time(timestr);
                    //fprintf(stderr, "%s, %llu\n",timestr.c_str(), u_time_t);
                }
            }
        }           
        return u_time_t;
    }

public:
    int fdopen(const char * devicePath,int is_sync)  {
        this->is_sync = is_sync;

        int res = sio.openUART(TTY_PATH);
        if(res <= 0 )
        {
            AERROR << "open serial port divce error ( please check hardware , "<<
            "or checkdevice driver was install? `lsmod | grep *timer*`" << std::endl;
            return false;
        }

        return true;
    }

    void fdclose()  {
        sio.closeUART();
    }

    time_t getTime()  {
        if (this->is_sync)
        {

            return this->get_uart_time();

        }else{
            time_t base = 0;
            while(base == 0)
            {
                base = this->get_uart_time();
            }
            return base;
        }
        
        AERROR << "get time failed ,may be devices not init or not detected" << std::endl;
        return -1;
    }
private:
    int is_sync;
    string dbuffer = "";
    SerialIO sio;

};

class IOTimerLogic{
private:
    std::thread thread_io;
    std::thread thread_uart;
    std::queue<int> queue_io;
    std::queue<time_t> queue_uart;
    std::mutex io_lock,uart_lock;

private:
    uint64_t calcCounter(time_t base1,time_t base0,uint64_t count,uint64_t diff)   {
        if ( count > 1000 && count < 2000) {
            time_t calc_value = count - (base1 - base0) * 1000 - diff;
            return calc_value;

        }else if (count > 2000)   {
            time_t calc_value =  count - (base1 - base0) * 1000;
            return calc_value;
        }
        return count;
    }

public:
    void init() {
        this->iodata.fdopen(IO_PATH);
        this->uarttime.fdopen(TTY_PATH,0);

    }

    void SyncIO() {
        while(*all_thread_run){
            base0 = this->uarttime.getTime();
            if(uart_lock.try_lock() ){
                this->queue_uart.push(base0);
                uart_lock.unlock();
            }
            usleep(1);
        }
    }

    void SyncUART()  {
        while(*all_thread_run){
            this->iodata.getIOData(&data[0],CHANNEL_LENGTH);
            if(io_lock.try_lock() ){
                this->queue_io.push(data[0].data);
                io_lock.unlock();
            }
            usleep(1);
        }
    }

    void StopThread()   {
        if (thread_uart.joinable()) {
            thread_uart.join();
        }
        if (thread_io.joinable()) {
            thread_io.join();
        }
    }


    void dispatchTime(bool *thread_is_run_)  {
        all_thread_run = thread_is_run_;
        thread_io = std::thread(&IOTimerLogic::SyncIO, this);
        thread_uart = std::thread(&IOTimerLogic::SyncUART, this);

        if (this->queue_uart.size() > 0){
            if( uart_lock.try_lock() )  {
                base0 = this->queue_uart.front();
                this->queue_uart.pop();
                uart_lock.unlock();
            }
        }else{
            base0 = 0;
        }
        int iotime = 0;
        while(1){
            if(*thread_is_run_ == false) {
                StopThread();
                return;
            }            
            //this->iodata.getIOData(&data[0],CHANNEL_LENGTH);
            
            if (this->queue_io.size() > 0){
                if( io_lock.try_lock() ){
                    iotime = this->queue_io.front();
                    this->queue_io.pop();
                    io_lock.unlock();
                }
            }
            if ( flag != (iotime & 2) ) {
                flag = iotime & 2;
                cur_counter = 0;
            }
            cur_counter ++;

            diff = cur_counter - 1000;            
            if (this->queue_uart.size() > 0){

                if( uart_lock.try_lock() )  {
                    base1 = this->queue_uart.front();
                    this->queue_uart.pop();
                    uart_lock.unlock();
                }
            }else{
                base1 = base1;
            }
            if (base1 != base0) {
                this->cur_counter = (int)this->calcCounter(base1,base0,cur_counter,diff);
                base0 = base1;
            }
#if( TIME_TS_DEBUG == 1)
            fprintf(stdout,"cur_counter:%d , diff: %ld ,base0:%lld \n",cur_counter,diff,base1);
#endif
            //AINFO<<"cur_counter:"<<cur_counter<<",diff:"<<diff<<" , base0:",base0<<"\n";
        }
    }

    void close()  {
        this->iodata.fdclose();
        this->uarttime.fdclose();
    }

    time_t get_time()   {
        return this->base0;
    }

    int get_utime()  {
        return this->cur_counter;
    }

private:
    IOData iodata;
    UartTime uarttime;
    SEND_PACK data[CHANNEL_LENGTH];
    uint8_t flag = 0;
    int cur_counter = 0;
    time_t base0 = 0;
    time_t base1 = 0;
    int diff = 0;
    bool *all_thread_run;
};

static IOTimerLogic logic;

TimeTsServices* TimeTsServices::GetInstance() {
    if(instance_ == NULL) {
        logic.init();
        instance_ = new TimeTsServices();
        instance_->StartThread();
    }
    return instance_;
}

void TimeTsServices::ReleaseTimeTsServices() {
    if(instance_ != NULL) {
        logic.close();
        instance_->StopThread();
        delete instance_;
        instance_ = NULL;
        std::cout << "Destroy Singleton Object. " << std::endl;
    }
}

void TimeTsServices::StopThread() {
    thread_is_run_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void TimeTsServices::StartThread() {
    thread_is_run_ = true;
    thread_ = std::thread(&TimeTsServices::SyncTs, this);
}


void TimeTsServices::SyncTs() {
    std::cout << "---SyncTs enter---" << std::endl;

    logic.dispatchTime(&this->thread_is_run_);

    std::cout << "---SyncTs exit---" << std::endl;
}

void TimeTsServices::GetSyncInfo(uint32_t* sec, uint32_t* nsec) {

    time_t tm_sync = logic.get_time();
    uint32_t count = logic.get_utime();

    *sec = (uint32_t)tm_sync;
    *nsec = count;
}

}  // namespace camera
}  // namespace drivers
