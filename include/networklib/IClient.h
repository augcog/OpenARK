#pragma once
#include <string>

namespace NetworkLib
{
	class IClient abstract
	{
	public:
		virtual ~IClient()
		{
		};
		virtual bool HasMessages() abstract = 0;
		virtual void Send(const std::string& message) abstract = 0;
		virtual std::string PopMessage() abstract = 0;
	};
}
