#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstdio>

#pragma comment(lib, "Ws2_32.lib")

#define UDP_PORT 5053
#define NUM_FLOATS 45  // 15 bones * 3 (x,y,z)

int main() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;  // or inet_addr("127.0.0.1")

    bind(sock, (sockaddr*)&addr, sizeof(addr));

    float buffer[NUM_FLOATS];

    while (true) {
        int bytesReceived = recvfrom(sock, (char*)buffer, sizeof(buffer), 0, nullptr, nullptr);

        if (bytesReceived == NUM_FLOATS * sizeof(float)) {
            // buffer[0..2]   = Palm    (x, y, z)
            // buffer[3..5]   = Thumb1
            // buffer[6..8]   = Thumb2
            // buffer[9..11]  = Index1
            // ... etc, matching bone_order in Python

            printf("Palm rot: %.3f %.3f %.3f\n", buffer[0], buffer[1], buffer[2]);
        }
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}