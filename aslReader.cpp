#include <winsock2.h> // Winsock header
#include <ws2tcpip.h> // Extended Winsock stuff, needed for some helper functions
#include <cstdio> // Using printf not cout for this one
#include <vector> // Vector cuz i dont like arrays
#include <string> // String

#pragma comment(lib, "Ws2_32.lib") // Autolink Winsock if using MSVC
// Not needed if runing g++ tho

// Note, the .bat file only works with g++, need smth different if not using g++

#define UDP_PORT 5053 // Output port
#define NUM_BONES 15 // Number of bones
#define NUM_FLOAT 45 // Would perefer double byt python packs in floats

#define TRACKED_BONE 0 // Goes from 0 to 14

int frameCounter;

const std::vector<std::string> BONE_NAMES = {
    "Palm", "Thumb1", "Thumb2",
    "Index1", "Index2", "Index3",
    "Middle1", "Middle2", "Middle3",
    "Ring1", "Ring2", "Ring3",
    "Pinky1", "Pinky2", "Pinky3"
}; // Use this too to track


void printBoneRotation(const std::vector<std::vector<float>>& rotations, int boneIndex) {
    if (boneIndex < 0 || boneIndex >= (int)rotations.size()) {
        printf("Invalid bone index: %d\n", boneIndex);
        return;
    }

    const auto& rotation = rotations[boneIndex];
    printf("%s rot: %.3f %.3f %.3f\n", BONE_NAMES[boneIndex].c_str(), rotation[0], rotation[1], rotation[2]);
} // Debugging function, used to get the information for finding rotational data


int main() {
    WSADATA wsaData; // Winsock fills this with version and implimntation info, needed by API as seen below
    WSAStartup(MAKEWORD(2, 2), &wsaData); // Initalize the Winsock library

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // Create UDP socket

    sockaddr_in addr{}; // To hold address
    addr.sin_family = AF_INET; // Specifies the address family, in this case IPv4
    addr.sin_port = htons(UDP_PORT); // Sets the port
    addr.sin_addr.s_addr = INADDR_ANY;  // Binds to all local network interfaces

    bind(sock, (sockaddr*)&addr, sizeof(addr)); // Binds socket to the port

    std::vector<float> buffer(NUM_FLOAT); // Buffer for the rotational data

    while (true) {
        std::vector<std::vector<float>> rotations{}; // 2D vector to actually hold the data
        int bytesReceived = recvfrom(sock, (char*)buffer.data(), buffer.size() * sizeof(float), 0, nullptr, nullptr); 
        frameCounter++;
        // Block until package recieved, then copy raw info into buffer
        for (int i{}; i < NUM_FLOAT; i+=3){
            float tempX = buffer[i];
            float tempY = buffer[i+1];
            float tempZ = buffer[i+2];
            // Temp vars to hold rotations
            rotations.push_back({tempX, tempY, tempZ});
            // Add rotations to a more conveniant vector
        }
        // For loop to place everything into a more convenient structure

        if (bytesReceived != NUM_FLOAT * sizeof(float)) {
            printf("Error has occured with data receiving");
        }
        if (frameCounter % 30 == 0){
            printBoneRotation(rotations, TRACKED_BONE);
        }
    }

    closesocket(sock); // Close the socket
    WSACleanup(); // Un-initialize Winsock
    return 0;
}