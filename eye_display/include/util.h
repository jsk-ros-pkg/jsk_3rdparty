#ifndef __UTIL_H__
#define __UTIL_H__

#include <iterator>
#include <sstream>
#include <list>

std::string trim(const std::string& s) {
  auto start = std::find_if_not(s.begin(), s.end(), ::isspace);
  auto end = std::find_if_not(s.rbegin(), s.rend(), ::isspace).base();
  if (start >= end) return "";
  return std::string(start, end);
}

std::list<std::string> splitComma(const std::string& input) {
  std::list<std::string> result;
  std::stringstream ss(input);
  std::string item;

  while (std::getline(ss, item, ',')) {
    result.push_back(trim(item));
  }

  // just in case where last element is blank
  if (!input.empty() && input.back() == ',') {
    result.push_back("");  // add null element
  }

  return result;
}

bool splitKeyValue(const std::string& message, std::string& key, std::string& value) {
  size_t colonPos = message.find(':');
  if (colonPos == std::string::npos) {
    return false;  // failed if ":" is not found
  }

  // trim key
  size_t keyStart = 0;
  size_t keyEnd = colonPos;
  while (keyEnd > keyStart && std::isspace(message[keyEnd - 1])) --keyEnd;

  // trim value
  size_t valStart = colonPos + 1;
  while (valStart < message.size() && std::isspace(message[valStart])) ++valStart;
  size_t valEnd = message.size();
  while (valEnd > valStart && std::isspace(message[valEnd - 1])) --valEnd;

  key = message.substr(keyStart, keyEnd - keyStart);
  value = message.substr(valStart, valEnd - valStart);
  return true;
}

bool parseXY(const std::string& input, double& x, double& y) {
  // remove spaces
  std::string cleaned;
  for (char c : input) {
    // convert camma to spaces
    if (c == ',') {
      cleaned += ' ';
    } else {
      cleaned += c;
    }
  }

  std::istringstream iss(cleaned);
  return static_cast<bool>(iss >> x >> y);  // return true if there are x and y
}

template <typename T>
std::string joinVector(const std::vector<T>& vec, const std::string& sep = ", ") {
  std::ostringstream oss;
  for (size_t i = 0; i < vec.size(); ++i) {
    oss << vec[i];
    if (i + 1 < vec.size()) oss << sep;
  }
  return oss.str();
}

#endif // __UTIL_H__
