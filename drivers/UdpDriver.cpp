#include "drivers_sonar_tritech/UdpDriver.hpp"

UDPDriver::UDPDriver(bool debug) : Driver(1024, false, debug){};

bool UDPDriver::init(std::string udpServer, int port) {
   openUDP(udpServer, port);
   return isValid();
}

void UDPDriver::sendSonarBeam(const base::samples::SonarBeam& beam) {
   std::stringstream s;
   s << beam.time.toSeconds() << " " << beam.bearing.rad << " "
     << beam.getSpatialResolution() << " " << beam.beam.size() << " ";
   if (debug_) {
      std::cout << "time sec:" << beam.time.toSeconds() << std::endl;
      std::cout << "beam.bearing.rad:" << beam.bearing.rad << std::endl;

      std::cout << "beam.getSpatialResolution():" << beam.getSpatialResolution()
                << std::endl;
      std::cout << "beam.beam.size():" << beam.beam.size() << std::endl;
      std::cout << "beamwidth_horizontal:" << beam.beamwidth_horizontal
                << std::endl;
      std::cout << "beamwidth_vertical:" << beam.beamwidth_vertical
                << std::endl;
   }

   for (size_t i = 0; i < beam.beam.size(); i++) {
      s << (unsigned int)beam.beam[i] << " ";
   }
   s << "\n";
   size_t len = s.str().size();
   if (!isValid()) {
      if (debug_) {
         std::cout << len << " " << s.str() << std::endl;
      }
      std::cout << "Warning socket is no open (yet?)" << std::endl;
   } else {
      std::cout << len << " " << s.str() << std::endl;
      writePacket((const uint8_t*)s.str().c_str(), len);
   }
}