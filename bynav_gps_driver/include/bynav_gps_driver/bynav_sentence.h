#ifndef BYNAV_SENTENCE_H_
#define BYNAV_SENTENCE_H_

#include <string>
#include <vector>

namespace bynav_gps_driver {

struct BynavSentence {
  std::string id;
  std::vector<std::string> header;
  std::vector<std::string> body;
};
} // namespace bynav_gps_driver

#endif // BYNAV_SENTENCE_H_
