#pragma once

#include <string>

namespace rfs {

void setFilterScanId(int scanId);
int filterScanId();
void logFilterAction(const std::string &filterName, const std::string &method,
                     const std::string &details = "");

} // namespace rfs
