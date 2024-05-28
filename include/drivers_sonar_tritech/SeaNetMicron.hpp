#ifndef _SEANETMICRON_H_
#define _SEANETMICRON_H_

#include "RigidBodyState.hpp"
#include "SeaNet.hpp"
#include "SeaNetTypesInternal.hpp"
#include "Sonar.hpp"

namespace sea_net {
struct HeadConfigPacket;

class Micron : public SeaNet {
  public:
   Micron(bool debug = false);
   ~Micron();
   void configure(const MicronConfig &config, uint32_t timeout,
                  bool stare_llm = false);
   void decodeSonar(base::samples::Sonar &beam);
   void decodeEchoSounder(base::samples::RigidBodyState &state);

  private:
   HeadCommand head_config;
   double speed_of_sound;
};
};  // namespace sea_net

#endif
