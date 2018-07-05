#pragma once
#include <string>
#include <utility>

namespace NetworkLib
{
	typedef std::pair<std::string, uint32_t> ClientMessage;

	class IServer abstract
	{
	public:
		virtual ~IServer()
		{
		};
		virtual bool HasMessages() abstract = 0;
		virtual void SendToClient(const std::string& message, uint32_t clientID) abstract = 0;
		virtual ClientMessage PopMessage() abstract = 0;
		virtual size_t GetClientCount() abstract = 0;
		virtual uint32_t GetClientIdByIndex(size_t) abstract = 0;
	};
}
