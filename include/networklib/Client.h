#pragma once
#include "Constants.h"
#include "Statistics.h"

#include "LockedQueue.h"

#include <boost/asio.hpp>

#include <array>
#include <thread>
#include "IClient.h"


using boost::asio::ip::udp;

namespace NetworkLib
{
	class Client : public IClient
	{
	public:
		Client(std::string host, unsigned short server_port, unsigned short local_port = 0);
		virtual ~Client();

		void Send(const std::string& message) override;

		bool HasMessages() override;;

		std::string PopMessage() override;;

	private:
		// Network send/receive stuff
		boost::asio::io_service io_service;
		udp::socket socket;
		udp::endpoint server_endpoint;
		udp::endpoint remote_endpoint;
		std::array<char, NetworkBufferSize> recv_buffer;
		std::thread service_thread;

		// Queues for messages
		LockedQueue<std::string> incomingMessages;

		void start_receive();
		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
		void run_service();

		Client(Client&); // block default copy constructor

		// Statistics
		Statistics statistics;
	};
}
