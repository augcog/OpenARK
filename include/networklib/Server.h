#pragma once
#include "Constants.h"
#include "Statistics.h"

#include "LockedQueue.h"

#include <boost/asio.hpp>

#include <array>
#include <map>
#include <thread>
#include <atomic>
#include "IServer.h"

using boost::asio::ip::udp;

typedef std::map<uint32_t, udp::endpoint> ClientList;
typedef ClientList::value_type Client;

namespace NetworkLib
{
	class Server : public IServer
	{
	public:
		explicit Server(unsigned short local_port);
		virtual ~Server();

		bool HasMessages() override;
		ClientMessage PopMessage() override;

		void SendToClient(const std::string& message, uint32_t clientID) override;
		void SendToAllExcept(const std::string& message, uint32_t clientID);
		void SendToAll(const std::string& message);

		size_t GetClientCount() override;
		uint32_t GetClientIdByIndex(size_t index) override;

		const Statistics& GetStatistics() const { return statistics; };
		std::vector<std::function<void(uint32_t)>> clientDisconnectedHandlers;
	private:
		// Network send/receive stuff
		boost::asio::io_service io_service;
		udp::socket socket;
		udp::endpoint server_endpoint;
		udp::endpoint remote_endpoint;
		std::array<char, NetworkBufferSize> recv_buffer;
		std::thread service_thread;

		// Low-level network functions
		void start_receive();
		void handle_remote_error(boost::system::error_code error_code, udp::endpoint remote_endpoint);
		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);

		void handle_send(std::string /*message*/, const boost::system::error_code& /*error*/, std::size_t /*bytes_transferred*/)
		{
		}

		void run_service();

		// Client management
		int32_t get_or_create_client_id(udp::endpoint endpoint);
		void on_client_disconnected(int32_t id);

		void send(const std::string& message, udp::endpoint target);

		// Incoming messages queue
		LockedQueue<ClientMessage> incomingMessages;

		// Clients of the server
		ClientList clients;
		std::atomic_int32_t nextClientID;

		Server(Server&); // block default copy constructor

		// Statistics
		Statistics statistics;
	};
}
