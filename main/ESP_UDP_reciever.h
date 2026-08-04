#pragma once

#include <cstdint>

//Starts UDP-listening task
bool startUDPReceiver();

//Copies Pi command and returns false if no command received
bool getLatestPiCommand(
    float& forward,
    float& turn,
    uint32_t& commandAgeMs
);
