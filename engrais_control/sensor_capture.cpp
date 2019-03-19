//********************************************************************************************************

#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>

#include "src/sensor_capture_include/1_VelodyneCapture/VelodyneCapture.h"

#define PI 3.1415926535

using namespace std;

class Point3d{
    public:
        double x = -5000;
        double y = -5000;
        double z = -5000;

    public:
        Point3d(){}
        Point3d(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};
//--------------------------------------------------------------------------------------------------------
int main( int argc, char* argv[] ){

    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.21" );
    const unsigned short port = 2368;

    VLP16Capture capture( address, port );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    while( capture.isRun() ){
        // Capture One Rotation Data
        std::vector<Laser> lasers;
        capture >> lasers;

        if( lasers.empty() ){
            continue;
        }

        // Access to Laser Data
        /*for( const Laser& laser : lasers ){

            if(laser.distance == 0)
                continue;

            // Sort Laser Data ( 0 degree -> 359 degree, 0 id -> n id )
            std::sort( lasers.begin(), lasers.end() );

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
        }*/

        // Convert to 3-dimension Coordinates
        std::vector<Point3d> buffer;
        for( const Laser& laser : lasers ){
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * PI / 180.0;
            const double vertical = laser.vertical * PI / 180.0;
         
            double x = static_cast<double>( ( (distance / 100.0) * std::cos( vertical ) ) * std::sin( azimuth ) );
            double y = static_cast<double>( ( (distance / 100.0) * std::cos( vertical ) ) * std::cos( azimuth ) );
            double z = static_cast<double>( ( (distance / 100.0) * std::sin( vertical ) ) );
         
            if( x == 0.0f && y == 0.0f && z == 0.0f ){
                x = std::numeric_limits<double>::infinity();
                y = std::numeric_limits<double>::infinity();
                z = std::numeric_limits<double>::infinity();
            }
            
            buffer.push_back( Point3d( x, y, z ) );
        }

        cout << "Vector size: " <<  buffer.size() << " {" << endl;
        /*for(Point3d p : buffer){
            cout << "\tPoint3d : [ x: " << p.x << ", y: " << p.y << ", z " << p.z << " ]" << endl;
        }
        cout  << "}" << endl;*/
        //exit(1);
    }

    return 0;
}

//********************************************************************************************************