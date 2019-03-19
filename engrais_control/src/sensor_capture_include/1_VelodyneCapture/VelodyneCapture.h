//********************************************************************************************************

// VelodyneCapture
//
// VelodyneCapture is the capture class to retrieve the laser data from Velodyne sensors using Boost.Asio and PCAP.
// VelodyneCapture will be able to retrieve lasers infomation about azimuth, vertical and distance that capture at one rotation.
// This class supports direct capture form Velodyne sensors, or capture from PCAP files.
// ( This class only supports VLP-16 and HDL-32E sensor, and not supports Dual Return mode. )
//
// If direct capture from sensors, VelodyneCapture are requires Boost.Asio and its dependent libraries ( Boost.System, Boost.Date_Time, Boost.Regex ).
// Please define HAVE_BOOST in preprocessor.
//
// If capture from PCAP files, VelodyneCapture are requires PCAP.
// Please define HAVE_PCAP in preprocessor.
//
// This source code is licensed under the MIT license. Please see the License in License.txt.
// Copyright (c) 2017 Tsukasa SUGIURA
// t.sugiura0204@gmail.com

#ifndef VELODYNE_CAPTURE
#define VELODYNE_CAPTURE

#include <iostream>
#include <string>
#include <sstream>

#include <thread>
#include <atomic>
#include <mutex>

#include <queue>
#include <vector>
#include <cassert>

#include <cstdint>
#include <chrono>
#include <iomanip>

#include <algorithm>
#include <functional>

#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#define EPSILON 0.001

struct Laser {
    double azimuth;
    double vertical;
    float distance;
    unsigned char intensity;
    unsigned char id;
    long long time;

    const bool operator < ( const struct Laser& laser ){
        if( azimuth == laser.azimuth ){
            return id < laser.id;
        }
        else{
            return azimuth < laser.azimuth;
        }
    }
};


//--------------------------------------------------------------------------------------------------------
class VLP16Capture {

    private:

        boost::asio::io_service ioservice;
        boost::asio::ip::udp::socket* socket = nullptr;
        boost::asio::ip::address address;
        unsigned short port = 2368;

        std::thread* thread = nullptr;
        std::atomic_bool run = { false };
        std::mutex mutex;
        std::queue<std::vector<Laser>> queue;

        int MAX_NUM_LASERS = 16;
        std::vector<double> lut = { -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0 };
        double time_between_firings = 2.304;
        double time_half_idle = 18.432;
        double time_total_cycle = 55.296 * 2;

        static const int LASER_PER_FIRING = 32;
        static const int FIRING_PER_PKT = 12;

        #pragma pack(push, 1)
        typedef struct LaserReturn
        {
            uint16_t distance;
            uint8_t intensity;
        } LaserReturn;
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct FiringData
        {
            uint16_t blockIdentifier;
            uint16_t rotationalPosition;
            LaserReturn laserReturns[LASER_PER_FIRING];
        };
        #pragma pack(pop)

        #pragma pack(push, 1)
        struct DataPacket
        {
            FiringData firingData[FIRING_PER_PKT];
            uint32_t gpsTimestamp;
            uint8_t mode;
            uint8_t sensorType;
        };
        #pragma pack(pop)

    public:
        //------------------------------------------------------------------------------------------------
        VLP16Capture();
        //------------------------------------------------------------------------------------------------
        VLP16Capture( const boost::asio::ip::address& address, const unsigned short port = 2368 );
        //------------------------------------------------------------------------------------------------
        ~VLP16Capture();
        //------------------------------------------------------------------------------------------------
        const bool open( const boost::asio::ip::address& address, const unsigned short port = 2368 );
        //------------------------------------------------------------------------------------------------
        const bool isOpen();
        //------------------------------------------------------------------------------------------------
        const bool isRun();
        //------------------------------------------------------------------------------------------------
        void close();
        //------------------------------------------------------------------------------------------------
        void retrieve( std::vector<Laser>& lasers, const bool sort = false );
        //------------------------------------------------------------------------------------------------
        void operator >> ( std::vector<Laser>& lasers );
        //------------------------------------------------------------------------------------------------
        size_t getQueueSize();


    private:

        //------------------------------------------------------------------------------------------------
        void parseDataPacket( const DataPacket* packet, std::vector<Laser>& lasers, double& last_azimuth );
        //------------------------------------------------------------------------------------------------
        void captureSensor();
        //------------------------------------------------------------------------------------------------
};


//--------------------------------------------------------------------------------------------------------
inline VLP16Capture::VLP16Capture(){}
//--------------------------------------------------------------------------------------------------------
inline VLP16Capture::VLP16Capture( const boost::asio::ip::address& address, const unsigned short port ){ open( address, port ); }
//--------------------------------------------------------------------------------------------------------
inline VLP16Capture::~VLP16Capture(){ close(); }
//--------------------------------------------------------------------------------------------------------
inline const bool VLP16Capture::isOpen(){ std::lock_guard<std::mutex> lock( mutex ); return socket && socket->is_open(); }
//--------------------------------------------------------------------------------------------------------
inline const bool VLP16Capture::isRun(){ std::lock_guard<std::mutex> lock( mutex ); return ( run || !queue.empty() ); }
//--------------------------------------------------------------------------------------------------------
inline void VLP16Capture::operator >> ( std::vector<Laser>& lasers ) { retrieve( lasers, false ); }
//--------------------------------------------------------------------------------------------------------
inline size_t VLP16Capture::getQueueSize() { std::lock_guard<std::mutex> lock( mutex ); return queue.size(); }
//--------------------------------------------------------------------------------------------------------


#endif
//********************************************************************************************************
