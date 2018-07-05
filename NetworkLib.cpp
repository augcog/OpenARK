#include "NetworkLib.h"
#include <iostream>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/algorithm/string.hpp>

namespace NetworkLib {
    namespace Util {
        // Encode string to base64 string
        bool Base64Encode(const std::string& input, std::string* output) {
            try {
                using namespace boost::archive::iterators;
                using It = base64_from_binary<transform_width<std::string::const_iterator, 6, 8>>;
                auto tmp = std::string(It(std::begin(input)), It(std::end(input)));
                *output = tmp.append((3 - input.size() % 3) % 3, '=');
                return true;
            } catch (...) {
                return false;
            }
        }

        // Decode base64 string to string
        bool Base64Decode(const std::string& input, std::string* output) {
            try {
                using namespace boost::archive::iterators;
                using It = transform_width<binary_from_base64<std::string::const_iterator>, 8, 6>;
                *output = boost::algorithm::trim_right_copy_if(std::string(It(std::begin(input)), It(std::end(input))), [](char c) { return c == '\0'; });
                return true;
            } catch (...) {
                return false;
            } 
        }
    }

    Server::Server(unsigned short local_port) :
        socket(io_service, udp::endpoint(udp::v4(), local_port)),
		service_thread(&Server::run_service, this),
		nextClientID(0L)
	{ };

	Server::~Server()
	{
		io_service.stop();
		service_thread.join();
	}

	void Server::start_receive()
	{
		socket.async_receive_from(boost::asio::buffer(recv_buffer), remote_endpoint,
			[this](boost::system::error_code ec, std::size_t bytes_recvd){ this->handle_receive(ec, bytes_recvd); });
	}

	void Server::on_client_disconnected(int32_t id)
	{
		for (auto& handler : clientDisconnectedHandlers)
			if (handler)
				handler(id);
	}

	void Server::handle_remote_error(const boost::system::error_code error_code, const udp::endpoint remote_endpoint)
	{
		bool found = false;
		int32_t id;
		for (const auto& client : clients)
			if (client.second == remote_endpoint) {
				found = true;
				id = client.first;
				break;
			}
		if (found == false)
			return;

		clients.erase(id);
		on_client_disconnected(id);
	}

	void Server::handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred)
	{
		if (!error)
		{
			try {
				auto message = ClientMessage(std::string(recv_buffer.data(), recv_buffer.data() + bytes_transferred), get_or_create_client_id(remote_endpoint));
				if (!message.first.empty())
					incomingMessages.push(message);
			}
			catch (std::exception & ex) {
                std::cerr << "handle_receive: Error parsing incoming message:" << ex.what() << "\n";
			}
			catch (...) {
                std::cerr << "handle_receive: Unknown error while parsing incoming message\n";
			}
		}
		else
		{
            std::cerr << "handle_receive: error: " << error.message() << " while receiving from address " << remote_endpoint << "\n";
			handle_remote_error(error, remote_endpoint);
		}

		start_receive();
	}

	void Server::send(const std::string& message, udp::endpoint target_endpoint)
	{
		socket.send_to(boost::asio::buffer(message), target_endpoint);
	}

	void Server::run_service()
	{
		start_receive();
		while (!io_service.stopped()){
			try {
				io_service.run();
			}
			catch (const std::exception& e) {
                std::cerr << "Server: Network exception: " << e.what() << "\n";
			}
			catch (...) {
                std::cerr << "Server: Network exception: unknown";
			}
		}
	};

	int32_t Server::get_or_create_client_id(udp::endpoint endpoint)
	{
		for (const auto& client : clients)
			if (client.second == endpoint)
				return client.first;

		auto id = ++nextClientID;
		clients.insert(Client(id, endpoint));
		return id;
	};

	void Server::SendToClient(const std::string& message, uint32_t clientID)
	{
		try {
			send(message, clients.at(clientID));
		}
		catch (std::out_of_range) {
            std::cerr << __FUNCTION__": Unknown client ID ";
		}
	};

	void Server::SendToAllExcept(const std::string& message, uint32_t clientID)
	{
		for (auto client : clients)
			if (client.first != clientID)
				send(message, client.second);
	};

	void Server::SendToAll(const std::string& message)
	{
		for (auto client : clients)
			send(message, client.second);
	}

	size_t Server::GetClientCount()
	{
		return clients.size();
	}

	uint32_t Server::GetClientIdByIndex(size_t index)
	{
		auto it = clients.begin();
		for (int i = 0; i < index; i++)
			++it;
		return it->first;
	}
    std::vector<uint32_t> Server::GetClients()
    {
        std::vector<uint32_t> result;
		auto it = clients.begin();
        int sz = clients.size();
        for (int i = 0; i < sz; i++) {
            result.push_back(it->first);
            ++it;
        }
        return result;
    }

	ClientMessage Server::PopMessage() {
		return incomingMessages.pop();
	}

	bool Server::HasMessages()
	{
		return !incomingMessages.empty();
	};
}
