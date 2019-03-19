//********************************************************************************************************

#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>

#include "src/sensor_capture_include/1_VelodyneCapture/VelodyneCapture.h"


//--------------------------------------------------------------------------------------------------------
int main( int argc, char* argv[] ){
    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.21" );
    const unsigned short port = 2368;
    velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    while( capture.isRun() ){
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            continue;
        }

        // Access to Laser Data
        for( const velodyne::Laser& laser : lasers ){
            // Laser Azimuth ( degree )
            const double azimuth = laser.azimuth;
            std::cout << "azimuth : " << azimuth << "\n";

            // Laser Vertical ( degree )
            const double vertical = laser.vertical;
            std::cout << "vertical : " << vertical << "\n";

            // Laser Distance ( centimeter )
            const unsigned short distance = laser.distance;
            std::cout << "distance : " << distance << "\n";

            // Laser Intensity
            const unsigned int intensity = static_cast<unsigned int>( laser.intensity );
            std::cout << "intensity : " << intensity << "\n";

            // Laser ID ( VLP-16 : 0 - 15, HDL-32E : 0 - 31 )
            const unsigned int id = static_cast<unsigned int>( laser.id );
            std::cout << "id : " << id << "\n";

            // Laser TimeStamp ( microseconds )
            const long long timestamp = laser.time;
            std::cout << "timestamp : " << timestamp << "\n";

            std::cout << std::endl;
        }

        std::cout << lasers.size() << std::endl << std::endl << std::endl;
        exit(1);
    }

    return 0;
}

//********************************************************************************************************