#pragma once
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include "stdafx.h"

#include <winsock2.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#define SERVER "127.0.0.1"  //ip address of udp server
#define BUFLEN 512  //Max length of buffer
#define PORT 8051   //The port on which to listen for incoming data

/**
* UDP client.
* Used to communicate information with game engines such as Unity
*/
class UDPSender
{
public:
	/**
	* Constructs a new UPD sender.
	*/
	UDPSender();

	/**
	* Sends a message.
	* @param message the data to be sent
	*/
	int send(std::string message);

	/**
	* Close the current connection.
	*/
	int close() const;

	struct sockaddr_in si_other;
	int s, slen = sizeof(si_other);
	char buf[BUFLEN];
	char message[BUFLEN];
	WSADATA wsa;
};