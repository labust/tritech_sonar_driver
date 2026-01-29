#include "drivers_sonar_tritech/SeaNet.hpp"
// #include <base-logging/Logging.hpp>
#include <iostream>
#include "drivers_sonar_tritech/SeaNetTypesInternal.hpp"
#include "drivers_sonar_tritech/Timeout.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace sea_net {

SeaNet::SeaNet(DeviceType type, bool debug)
    : iodrivers_base::Driver(SEA_NET_MAX_PACKET_SIZE, true),
      device_type(type),
      has_pending_data(false),
      debug_(debug) {}

SeaNet::~SeaNet() {}

void SeaNet::openSerial(std::string const& port, int baudrate) {
   if (debug_) {
      std::cout << "DEBUG: opening serial port: " << port
                << " with baudrate: " << baudrate << std::endl;
   }
   has_pending_data = false;
   ::iodrivers_base::Driver::openSerial(port, baudrate);
}

void SeaNet::openURI(std::string const& uri) {
   has_pending_data = false;
   return iodrivers_base::Driver::openURI(uri);
}

void SeaNet::clear() {
   has_pending_data = false;
   return iodrivers_base::Driver::clear();
}

void SeaNet::reboot(int timeout) {
   if (debug_) {
      std::cout << "DEBUG: rebooting device" << std::endl;
   }
   std::vector<uint8_t> packet =
       SeaNetPacket::createPaket(device_type, mtReBoot, NULL, 0, debug_);
   writePacket(&packet[0], packet.size());
   clear();

   // now wait for an alive message
   waitForPacket(mtAlive, timeout);
}

void SeaNet::getVersion(VersionData& version, int timeout) {
   std::vector<uint8_t> send_packet =
       SeaNetPacket::createPaket(device_type, mtReBoot, NULL, 0, debug_);
   writePacket(&send_packet[0], send_packet.size());

   // wait until we received a mtVersionData package
   waitForPacket(mtVersionData, timeout);
   sea_net_packet.decodeVersionData(version);
}

void SeaNet::waitForPacket(PacketType type, int timeout) {
   if (debug_) {
      std::cout << "DEBUG: wait for packet type: " << type << " timeout "
                << timeout << std::endl;
   }
   iodrivers_base::Timeout time_out(timeout);
   while (type != readPacket(time_out.timeLeft()));
}

bool SeaNet::isFullDublex(int timeout) { return isFullDuplex(timeout); }

bool SeaNet::isFullDuplex(int timeout) {
   std::vector<uint8_t> packet =
       SeaNetPacket::createPaket(device_type, mtReBoot, NULL, 0, debug_);
   writePacket(&packet[0], packet.size());
   waitForPacket(mtBBUserData, timeout);
   BBUserData settings;
   sea_net_packet.decodeBBUserData(settings);
   return settings.full_duplex;
}

SeaNetPacket const& SeaNet::getSeaNetPacket() const { return sea_net_packet; }

std::string SeaNet::formattedNow() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto now_seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto epoch = now_seconds.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::seconds>(epoch);
    long duration = value.count();

    auto nanoseconds = now.time_since_epoch() - std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
    long nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(nanoseconds).count();

    std::stringstream ss;
    ss << duration << "." << std::setw(9) << std::setfill('0') << nanos;

    return ss.str();
}

sea_net::PacketType SeaNet::readPacket(int timeout) {
   uint8_t* buffer = sea_net_packet.getPacketPtr();
   size_t size = iodrivers_base::Driver::readPacket(
       buffer, SEA_NET_MAX_PACKET_SIZE, base::Time::fromMilliseconds(timeout));
   sea_net_packet.setSize(size);
   if (!sea_net_packet.validate())
      throw std::runtime_error(
          "extractPacket has extracted an invalid package!");  // this should
                                                               // never happen
   if (debug_) {
      std::cout << "DEBUG: read packet of type: "
                << sea_net_packet.getPacketType() << std::endl;
   }

   if (sea_net_packet.getPacketType() == mtHeadData) has_pending_data = false;

   return sea_net_packet.getPacketType();
}

void SeaNet::writeHeadCommand(HeadCommand& head_config, int timeout) {
   if (has_pending_data)
      throw std::runtime_error(
          "requestData() called and the corresponding receiveData() has not "
          "been called");

   iodrivers_base::Timeout time_out(timeout);
   std::vector<uint8_t> packet = SeaNetPacket::createPaket(
       device_type, mtHeadCommand, (uint8_t*)&head_config, sizeof(head_config),
       debug_);
   if (debug_) {
      std::cout << "DEBUG: Sent mtHeadCommand packet" << std::endl;
   }
   writePacket(&packet[0], packet.size(), time_out.timeLeft());

   // wait for an alive packet to check if the sonar is configured
   AliveData alive_data;
   bool received_alive = false;
   while (!alive_data.ready && alive_data.no_config &&
          !alive_data.config_send) {
      try {
         waitForPacket(mtAlive, time_out.timeLeft());
         std::cout << "Sent mtAlive completed" << std::endl;
      } catch (std::runtime_error e) {
         if (received_alive)
            throw std::runtime_error(
                "Configure failed: Configuration was not accepted by the "
                "device.");
         else
            throw std::runtime_error(
                "Configure failed: Device is not responding. Timeout too "
                "small?");
      }
      received_alive = true;
      sea_net_packet.decodeAliveData(alive_data);
   }
}

void SeaNet::requestData() {
   if (debug_) {
      std::cout << "DEBUG: write mtSendData packet" << std::endl;
   }
   has_pending_data = true;
   std::vector<uint8_t> packet = SeaNetPacket::createSendDataPaket(device_type);
   writePacket(&packet[0], packet.size(), getWriteTimeout());
}

bool SeaNet::hasPendingData() const { return has_pending_data; }

void SeaNet::receiveData(int timeout) {
   if (debug_) {
      std::cout << "DEBUG: waiting for a mtHeadData packet" << std::endl;
   }
   waitForPacket(mtHeadData, timeout);
}

int SeaNet::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
   if (debug_) {
      std::cout << "DEBUG: trying to extract package buffer size "
                << buffer_size << std::endl;
   }

   return SeaNetPacket::isValidPacket(buffer, buffer_size, debug_);
}

void SeaNet::setWriteTimeout(uint32_t timeout) {
   Driver::setWriteTimeout(base::Time::fromMicroseconds(timeout * 1000));
}

}  // namespace sea_net
