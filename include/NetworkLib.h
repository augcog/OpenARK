#pragma once
#include <string>
#include <utility>
#include <boost/asio.hpp>
#include <array>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <list>

using boost::asio::ip::udp;

namespace NetworkLib {

    static const int NetworkBufferSize = 65535;

    typedef std::map<uint32_t, udp::endpoint> ClientList;
    typedef ClientList::value_type Client;

	namespace Util
	{
		// Encode string to base64 string
		bool Base64Encode(const std::string& input, std::string* output);

		// Decode base64 string to string
		bool Base64Decode(const std::string& input, std::string* output);
	}

    // Simple mutex-guarded queue
    template<typename _T> class LockedQueue
    {
        private:
            std::mutex mutex;
            std::queue<_T> queue;
        public:
            void push(_T value)
            {
                std::unique_lock<std::mutex> lock(mutex);
                queue.push(value);
            };

            // Get top message in the queue
            // Note: not exception-safe (will lose the message)
            _T pop()
            {
                std::unique_lock<std::mutex> lock(mutex);
                _T value;
                std::swap(value, queue.front());
                queue.pop();
                return value;
            };

            bool empty() {
                std::unique_lock<std::mutex> lock(mutex);
                return queue.empty();
            }
    };

    typedef std::pair<std::string, uint32_t> ClientMessage;
    class IServer
	{
	public:
		virtual ~IServer() {};
		virtual bool HasMessages() = 0;
		virtual void SendToClient(const std::string& message, uint32_t clientID) = 0;
		virtual ClientMessage PopMessage() = 0;
		virtual size_t GetClientCount() = 0;
		virtual uint32_t GetClientIdByIndex(size_t) = 0;
		virtual std::vector<uint32_t> GetClients() = 0;
	};

	class Server : public IServer {
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
		std::vector<uint32_t> GetClients();

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
		void handle_remote_error(const boost::system::error_code error_code, const udp::endpoint remote_endpoint);
		void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
		void handle_send(std::string /*message*/, const boost::system::error_code& /*error*/, std::size_t /*bytes_transferred*/)	{}
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
	};
}
