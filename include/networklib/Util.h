#pragma once
#include <iostream>
namespace NetworkLib {
	namespace Util
	{
		//Encode string to base64 string
		bool Base64Encode(const std::string& input, std::string* output);

		//Decode base64 string to string
		bool Base64Decode(const std::string& input, std::string* output);
	}
}
