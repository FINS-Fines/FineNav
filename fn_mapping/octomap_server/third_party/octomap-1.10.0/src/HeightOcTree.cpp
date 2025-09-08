#include "octomap/HeightOcTree.h"
#include <algorithm>

namespace octomap {

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

void HeightOcTree::updateNodeHeight(OcTreeNodeHeight* node, const float height) const {
    if (isNodeOccupied(node)) { // 占据，方才更新高度
        node->setHeight(height);
    } else {
        node->setHeight(NAN);
    }
}

HeightOcTree::StaticMemberInitializer HeightOcTree::heightOcTreeMemberInit;

} // end namespace


