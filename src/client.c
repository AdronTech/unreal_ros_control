// Client side C/C++ program to demonstrate Socket programming
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#define PORT 8080

#if __BIG_ENDIAN__
#define htonll(x) (x)
#define ntohll(x) (x)
#else
#define htonll(x) (((uint64_t)htonl((x)&0xFFFFFFFF) << 32) | htonl((x) >> 32))
#define ntohll(x) (((uint64_t)ntohl((x)&0xFFFFFFFF) << 32) | ntohl((x) >> 32))
#endif

int main(int argc, char const *argv[])
{
    struct sockaddr_in address;
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    memset(&serv_addr, '0', sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }

    // send(sock , buffer , 8 , 0 );

    double value = 0;
    uint64_t temp;
    while (1)
    {
        // valread = read( sock , buffer, 1024);

        // uint64_t temp = ntohll(*(uint64_t *)buffer);

        // double value = *(double *)&temp;
        // printf("%f\n", value);

        value += 2;

        char *pointer = buffer;
        int nr = 12;

        *(uint16_t *)pointer = htons(nr);
        pointer += 2;

        for (int i = 0; i < nr; i++)
        {
            char text[100];
            sprintf(text, "joint%d", i);

            *(uint16_t *)pointer = htons(strlen(text) + 1);
            pointer += 2;

            strcpy(pointer, text);
            pointer += strlen(text) + 1;

            temp = htonll(*(uint64_t *)&value);
            *(uint64_t *)pointer = temp;
            pointer += 4;
            *(uint64_t *)pointer = temp;
            pointer += 4;
            *(uint64_t *)pointer = temp;
            pointer += 4;
            send(sock, buffer, pointer - buffer, 0);
            pointer = buffer;
        }

        sleep(1);
    }

    return 0;
}