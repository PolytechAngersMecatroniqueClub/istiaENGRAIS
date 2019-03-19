//********************************************************************************************************
#include "VelodyneCapture.h"

//--------------------------------------------------------------------------------------------------------
const bool VLP16Capture::open( const boost::asio::ip::address& address, const unsigned short port){
    // Check Running
    if( isRun() ){
        close();
    }

    // Set IP-Address and Port
    this->address = ( !address.is_unspecified() ) ? address : boost::asio::ip::address::from_string( "255.255.255.255" );
    this->port = port;

    // Create Socket
    try{
        socket = new boost::asio::ip::udp::socket( ioservice, boost::asio::ip::udp::endpoint( this->address, this->port ) );
    }
    catch( ... ){
        delete socket;
        socket = new boost::asio::ip::udp::socket( ioservice, boost::asio::ip::udp::endpoint( boost::asio::ip::address_v4::any(), this->port ) );
    }

    // Start IO-Service
    try{
        ioservice.run();
    }
    catch( const std::exception& e ){
        std::cerr << e.what() << std::endl;
        return false;
    }

    // Start Capture Thread
    run = true;
    thread = new std::thread( std::bind( &VLP16Capture::captureSensor, this ) );

    return true;
}
//--------------------------------------------------------------------------------------------------------
void VLP16Capture::close() {
    run = false;
    // Close Capturte Thread
    if( thread && thread->joinable() ){
        thread->join();
        thread->~thread();
        delete thread;
        thread = nullptr;
    }

    std::lock_guard<std::mutex> lock( mutex );
    #ifdef HAVE_BOOST
    // Close Socket
    if( socket && socket->is_open() ){
        socket->close();
        delete socket;
        socket = nullptr;
    }

    // Stop IO-Service
    if( ioservice.stopped() ){
        ioservice.stop();
        ioservice.reset();
    }
    #endif

    // Clear Queue
    std::queue<std::vector<Laser>>().swap( queue );
}
//--------------------------------------------------------------------------------------------------------
void VLP16Capture::retrieve( std::vector<Laser>& lasers, const bool sort ) {
    if( mutex.try_lock() ){
        if( !queue.empty() ){
            lasers = std::move( queue.front() );
            queue.pop();
            if( sort ){
                std::sort( lasers.begin(), lasers.end() );
            }
        }
        mutex.unlock();
    }
}
//--------------------------------------------------------------------------------------------------------
void VLP16Capture::parseDataPacket( const DataPacket* packet, std::vector<Laser>& lasers, double& last_azimuth ) {
	if( packet->sensorType != 0x21 && packet->sensorType != 0x22 ){
		throw( std::runtime_error( "This sensor is not supported" ) );
	}

	if( packet->mode != 0x37 && packet->mode != 0x38){
		throw( std::runtime_error( "Sensor can't be set in dual return mode" ) );
	}

	// Retrieve Unix Time ( microseconds )
	const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
	const std::chrono::microseconds epoch = std::chrono::duration_cast<std::chrono::microseconds>( now.time_since_epoch() );
	const long long unixtime = epoch.count();

	// Azimuth delta is the angle from one firing sequence to the next one
	double azimuth_delta = 0.0;
	if( packet->firingData[1].rotationalPosition < packet->firingData[0].rotationalPosition ){
		azimuth_delta = ( ( packet->firingData[1].rotationalPosition + 36000 ) - packet->firingData[0].rotationalPosition );
	}
	else{
		azimuth_delta = ( packet->firingData[1].rotationalPosition - packet->firingData[0].rotationalPosition );
	}

	// Processing Packet
	for( int firing_index = 0; firing_index < FIRING_PER_PKT; firing_index++ ){
		// Retrieve Firing Data
		const FiringData firing_data = packet->firingData[firing_index];
		for( int laser_index = 0; laser_index < LASER_PER_FIRING; laser_index++ ){
			// Retrieve Rotation Azimuth
			double azimuth = static_cast<double>( firing_data.rotationalPosition );
			double laser_relative_time = LASER_PER_FIRING * time_between_firings + time_half_idle* (laser_index / MAX_NUM_LASERS);

			azimuth += azimuth_delta * laser_relative_time / time_total_cycle;

			// Reset Rotation Azimuth
			if( azimuth >= 36000 ) {
				azimuth -= 36000;
			}

			// Complete Retrieve Capture One Rotation Data
			#ifndef PUSH_SINGLE_PACKETS
			if( last_azimuth > azimuth ){
				// Push One Rotation Data to Queue
				mutex.lock();
				queue.push( std::move( lasers ) );
				mutex.unlock();
				lasers.clear();
			}
			#endif

			#ifdef NO_EMPTY_RETURNS
			if( firing_data.laserReturns[laser_index].distance < EPSILON ){
				continue;
			}
			#endif

			Laser laser;
			laser.azimuth = azimuth / 100.0f;
			laser.vertical = lut[laser_index % MAX_NUM_LASERS];

			#ifdef USE_MILLIMETERS
			laser.distance = static_cast<float>( firing_data.laserReturns[laser_index].distance ) * 2.0f;
			#else
			laser.distance = static_cast<float>( firing_data.laserReturns[laser_index].distance ) * 2.0f / 10.0f;
			#endif

			laser.intensity = firing_data.laserReturns[laser_index].intensity;
			laser.id = static_cast<unsigned char>( laser_index % MAX_NUM_LASERS );

			#ifdef HAVE_GPSTIME
			laser.time = packet->gpsTimestamp + static_cast<long long>( laser_relative_time );
			#else
			laser.time = unixtime + static_cast<long long>( laser_relative_time );
			#endif

			lasers.push_back( laser );
			// Update Last Rotation Azimuth
			last_azimuth = azimuth;
		}
	}

	#ifdef PUSH_SINGLE_PACKETS
	// Push packet after processing
	mutex.lock();
	queue.push( std::move( lasers ) );
	mutex.unlock();
	lasers.clear();
	#endif
}
//--------------------------------------------------------------------------------------------------------
void VLP16Capture::captureSensor(){
    double last_azimuth = 0.0;
    std::vector<Laser> lasers;
    unsigned char data[1500];
    boost::asio::ip::udp::endpoint sender;

    while( socket->is_open() && ioservice.stopped() && run ){
        // Receive Packet
        boost::system::error_code error;
        const size_t length = socket->receive_from( boost::asio::buffer( data, sizeof( data ) ), sender, 0, error );
        if( error == boost::asio::error::eof ){
            break;
        }

        // Check IP-Address and Port
        if( sender.address() != address && sender.port() != port ){
            continue;
        }

        // Check Packet Data Size
        // Data Blocks ( 100 bytes * 12 blocks ) + Time Stamp ( 4 bytes ) + Factory ( 2 bytes )
        if( length != 1206 ){
            continue;
        }

        // Convert to DataPacket Structure
        // Sensor Type 0x21 is HDL-32E, 0x22 is VLP-16
        const DataPacket* packet = reinterpret_cast<const DataPacket*>( data );
        parseDataPacket(  packet, lasers, last_azimuth );

    }
    run = false;
}
//--------------------------------------------------------------------------------------------------------

//********************************************************************************************************