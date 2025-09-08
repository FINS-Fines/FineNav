#include "octomap/HeightOcTree.h"
#include <algorithm>

namespace octomap {

// node implementation  --------------------------------------
std::ostream& OcTreeNodeHeight::writeData(std::ostream &s) const {
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &height, sizeof(height)); // height

    return s;
}

std::istream& OcTreeNodeHeight::readData(std::istream &s) {
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &height, sizeof(height)); // height

    return s;
}

// tree implementation  --------------------------------------
HeightOcTree::HeightOcTree(double in_resolution)
: OccupancyOcTreeBase<OcTreeNodeHeight>(in_resolution) {
    heightOcTreeMemberInit.ensureLinking();
}

bool HeightOcTree::isNodeCollapsible(const OcTreeNodeHeight* node) const {
  // 首先调用基类的合并条件检查
  if (!OccupancyOcTreeBase<OcTreeNodeHeight>::isNodeCollapsible(node))
      return false;

  // 如果任何子节点是占据的，则不允许合并
  for (unsigned int i = 0; i < 8; ++i) {
      if (nodeChildExists(node, i)) {
          const OcTreeNodeHeight* child = getNodeChild(node, i);
          if (child->getLogOdds() > 0.0f) { // 占据的节点
              return false; // 不允许合并
          }
      }
  }

  return true; // 只有当所有子节点都是空闲时才允许合并
}

void HeightOcTree::updateNodeHeight(const point3d& value, float height) {
    OcTreeKey key;
    if (!coordToKeyChecked(value, key))
        return;

    if (std::isnan(height)) {
        // 高度为NAN，设置为空闲
        OcTreeNodeHeight* node = updateNode(key, false, false);
        if (node) {
            node->setHeight(NAN);
        }
    } else {
        // 高度不是NAN，设置为占据
        OcTreeNodeHeight* node = updateNode(key, true, false);
        if (node) {
            node->setHeight(height);
        }
    }
}

HeightOcTree::StaticMemberInitializer HeightOcTree::heightOcTreeMemberInit;

} // end namespace
