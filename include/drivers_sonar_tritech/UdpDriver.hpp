#include <iostream>
#include "Driver.hpp"
#include "SeaNetMicron.hpp"

class UDPDriver : public iodrivers_base::Driver {
  public:
   UDPDriver(bool debug = false);

   bool init(std::string udpServer, int port);
   void sendSonarBeam(const base::samples::SonarBeam& beam);

  private:
   virtual int extractPacket(uint8_t const* buffer, size_t buffer_size) const {
      // We don't read data here
      return 0;
   }
};