#pragma once

#include <string>

namespace rfs {

void setAssociationScanId(int scanId);
int associationScanId();
void logAssociation(const std::string &message);

} // namespace rfs
