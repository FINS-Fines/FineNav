#ifndef OCTOMAP_HEIGHT_OCTREE_H
#define OCTOMAP_HEIGHT_OCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <iostream>

namespace octomap {

// node definition
class OcTreeNodeHeight : public OcTreeNode {
public:
    OcTreeNodeHeight() : OcTreeNode(), height(NAN) {}
    OcTreeNodeHeight(const OcTreeNodeHeight& rhs) : OcTreeNode(rhs), height(rhs.height) {}

    bool operator==(const OcTreeNodeHeight& rhs) const{
      return (rhs.value == value && rhs.height == height);
    }

    void copyData(const OcTreeNodeHeight& from){
      OcTreeNode::copyData(from);
      height = from.getHeight();
    }

    // height
    inline bool isHeightSet() const { return !std::isnan(height); }
    inline float getHeight() const { return height; }
    inline void setHeight(const float h) { height = h; }

    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

protected:
    float height;
};


// tree definition
// 由于树中不存储每个node的深度，无法保证只有达到树的最大深度的叶子节点才能设置高度
// 需要用户在调用时确认
class HeightOcTree : public OccupancyOcTreeBase <OcTreeNodeHeight> {
public:
    /// Default constructor, sets resolution of leafs
    HeightOcTree(double resolution);

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    HeightOcTree* create() const {return new HeightOcTree(resolution); }
    std::string getTreeType() const {return "HeightOcTree";}

    // 防止合并中高度信息丢失
    virtual bool isNodeCollapsible(const OcTreeNodeHeight* node) const override;

    //! Update node with height information
    void updateNodeHeight(OcTreeNodeHeight* node, float height) const;

    protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        HeightOcTree* tree = new HeightOcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

      /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
      void ensureLinking() {}
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer heightOcTreeMemberInit;

};

} // end namespace

#endif

