#pragma

#include <cstdint>

//Starts UDP-listening task
bool startUDPReceiver();

//Copies Pi command and returns fals if no command recieved
bool getLatestPiComman(
    float& forward,
    float& turn,
    uint32_t& commandAgeMs
);