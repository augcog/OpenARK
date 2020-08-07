/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 All rights reserved.

 This is the Author's implementation of BRISK: Binary Robust Invariant
 Scalable Keypoints [1]. Various (partly unpublished) extensions are provided,
 some of which are described in [2].

 [1] Stefan Leutenegger, Margarita Chli and Roland Siegwart. BRISK: Binary
     Robust Invariant Scalable Keypoints. In Proceedings of the IEEE
     International Conference on Computer Vision (ICCV), 2011.

 [2] Stefan Leutenegger. Unmanned Solar Airplanes: Design and Algorithms for
     Efficient and Robust Autonomous Operation. Doctoral dissertation, 2014.

 This file is part of BRISK.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BRISK_COMMAND_LINE_PARSER_H_
#define BRISK_COMMAND_LINE_PARSER_H_

#include <fstream>  // NOLINT
#include <iomanip>
#include <iostream>  //NOLINT
#include <list>
#include <vector>
#include <map>
#include <stdexcept>

namespace brisk {

/// \brief A simple parser of the terminal application arguments.
class CommandLineParser {
 public:
  enum ArgType { String, Num, Bool };
  struct ArgSpec {
    ArgType type;
    std::string name;
    std::string value;
    std::string description;
  };
  struct OptionSpec {
    ArgType type;
    std::string value;
    std::string description;
  };

  bool setArgs(const std::vector<ArgSpec>& args) {
    args_ = args;
    return true;
  }
  bool setOptions(const std::map<std::string, OptionSpec>& options) {
    options_ = options;
    optionsDefault_ = options;  // Backup for display.
    return true;
  }
  bool setInfo(std::string info) {
    info_ = info;
    return true;
  }

  bool parse(int argc, char** argv) {
    programName = argv[0];
    if (size_t(argc) < args_.size() + 1) {
      printUsage();
      return false;
    }
    for (size_t i = 1; i < args_.size() + 1; ++i) {
      args_[i].value = std::string(argv[i]);
    }
    for (size_t i = args_.size() + 1; i < size_t(argc); ++i) {
      std::string arg(argv[i]);
      size_t start = arg.find("--");
      if (start == std::string::npos) {
        start = arg.find("-");
        start += 1;
      } else {
        start += 2;
      }
      if (start == std::string::npos) {
        printUsage();
        return false;
      }
      size_t end = arg.find("=");
      std::string option;
      std::string value;
      if (end == std::string::npos) {
        // Syntax with different argv entry.
        option = arg.substr(start);
        if (i + 1 == size_t(argc)) {
          // No value.
        } else {
          // Get from next element.
          if (options_.find(option) != options_.end()) {
            value = std::string(argv[i + 1]);
            if (options_.at(option).type == Bool &&
                !(value == "true" || value == "false")) {
              value = "true";  // no bool means true
            } else {
              i++;  // Skip to next.
            }
          }
        }
      } else {
        // Syntax with =.
        option = arg.substr(start, end - start);
        value = arg.substr(end + 1);
      }
      if (option == "help") {
        printHelp();
        return false;
      } else {
        if (options_.find(option) == options_.end()) {
          std::cout << "Option --" << option << " not recognized." << std::endl;
          printUsage();
          return false;
        }
        options_.at(option).value = value;
      }
    }

    return true;
  }
  void printUsage() {
    std::cout << "Usage: " << programName;
    for (auto& iter : args_) {
      std::cout << iter.name << " ";
    }
    std::cout << "[options]" << std::endl;
    printHelp();
  }
  void printHelp() {
    std::cout << info_ << std::endl;
    if (args_.size() > 0) std::cout << "arguments:" << std::endl;
    for (auto& iter : args_) {
      std::cout << "  " << iter.name << ": " << iter.description << " "
                << std::endl;
    }
    std::cout << "options:" << std::endl;
    for (auto& iter : optionsDefault_) {
      std::cout << "  --" << iter.first << ": " << iter.second.description
                << " Default value = " << iter.second.value << std::endl;
    }
    std::cout << "  --help: Displays help." << std::endl;
  }

  bool optionAsBool(std::string option) const {
    if (options_.find(option) == options_.end()) {
      throw std::runtime_error("option --" + option + " does not exist");
      return false;
    }
    if (options_.at(option).type != ArgType::Bool) {
      throw std::runtime_error("option --" + option + " not bool");
      return false;
    }
    return options_.at(option).value.size() == 0 ||
           options_.at(option).value == "true";
  }

  float optionAsNum(std::string option) const {
    if (options_.find(option) == options_.end()) {
      throw std::runtime_error("option --" + option + " does not exist");
      return 0.0;
    }
    if (options_.at(option).type != ArgType::Num) {
      throw std::runtime_error("option --" + option + " not number");
      return 0.0;
    }
    float retVal;
    std::stringstream(options_.at(option).value) >> retVal;
    return retVal;
  }

  std::string optionAsString(std::string option) const {
    if (options_.find(option) == options_.end()) {
      throw std::runtime_error("option --" + option + " does not exist");
      return "";
    }
    if (options_.at(option).type != ArgType::String) {
      throw std::runtime_error("option --" + option + " not string");
      return "";
    }
    return options_.at(option).value;
  }

  bool argAsBool(size_t idx) const {
    if (idx >= args_.size()) {
      throw std::runtime_error("arg does not exist");
      return false;
    }
    if (args_[idx].type != ArgType::Bool) {
      throw std::runtime_error("arg not bool");
      return false;
    }
    return args_[idx].value.size() == 0 || args_[idx].value == "true";
  }

  float argAsNum(size_t idx) const {
    if (idx >= args_.size()) {
      throw std::runtime_error("arg does not exist");
      return 0.0;
    }
    if (args_[idx].type != ArgType::Num) {
      throw std::runtime_error("arg not number");
      return 0.0;
    }
    float retVal;
    std::stringstream(args_[idx].value) >> retVal;
    return retVal;
  }

  std::string argAsString(size_t idx) const {
    if (idx >= args_.size()) {
      throw std::runtime_error("arg does not exist");
      return "";
    }
    if (args_[idx].type != ArgType::String) {
      throw std::runtime_error("arg not string");
      return "";
    }
    return args_[idx].value;
  }

 private:
  std::vector<ArgSpec> args_;
  std::map<std::string, OptionSpec> options_;
  std::map<std::string, OptionSpec> optionsDefault_ = options_;
  std::string info_;
  std::string programName = "";
};

}  // namespace brisk

#endif  // BRISK_COMMAND_LINE_PARSER_H_
