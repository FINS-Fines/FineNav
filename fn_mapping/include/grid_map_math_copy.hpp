#include "GridMap/GridMapMath.hpp"

/*!
 * Gets the vector from the center of the map to the origin
 * of the map data structure.
 * @param[out] vectorToOrigin the vector from the center of the map
 *             the origin of the map data structure.
 * @param[in] mapLength the lengths in x and y direction.
 * @return true if successful.
 */
inline bool getVectorToOrigin(Vector& vectorToOrigin, const Length& mapLength)
{
  vectorToOrigin = (0.5 * mapLength).matrix();
    //vectorToOrigin是gridmap的中心点(m)
  return true;
}

/*!
 * Gets the vector from the center of the map to the center
 * of the first cell of the map data.
 * @param[out] vectorToFirstCell the vector from the center of
 *             the cell to the center of the map.
 * @param[in] mapLength the lengths in x and y direction.
 * @param[in] resolution the resolution of the map.
 * @return true if successful.
 */
inline bool getVectorToFirstCell(Vector& vectorToFirstCell,
                                 const Length& mapLength,
                                 const double& resolution)
{
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);

  // Vector to center of cell.
  //vectorToFirstCell是从map center到左上角点cell中心的偏移(m)
    //该cell也是在内存中顺序存储的第一个cell，同样在内存块的左上角点
  vectorToFirstCell = (vectorToOrigin.array() - 0.5 * resolution).matrix();
  return true;
}

// 图像坐标系到地图左上角坐标系，XY轴分别反转
inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
{
  return -Eigen::Matrix2i::Identity();
}

inline Eigen::Matrix2i getMapFrameToBufferOrderTransformation()
{
    //单位矩阵转置后还是单位矩阵
  return getBufferOrderToMapFrameTransformation().transpose();
}

inline bool checkIfStartIndexAtDefaultPosition(
  const Index& bufferStartIndex)
{
    //没有移动
  return ((bufferStartIndex == 0).all());
}

/**
 * 输入：内存块中的坐标cell
 * 中间量：unwrappedIndex是cell到内存坐标系原点（左上角，X正向下，Y正向右）的偏移（cell），该index没有考虑滚动的
 * 输出：index到以内存左上角为原点，X正向上，Y正向左的坐标系的转换关系
 */
inline Vector getIndexVectorFromIndex(
    const Index& index,
    const Size& bufferSize,
    const Index& bufferStartIndex)
{
  Index unwrappedIndex;
    //输入的index表示内存块中的坐标cell，index<bufferSize
  unwrappedIndex = getIndexFromBufferIndex(index, bufferSize,
                                           bufferStartIndex);
    //unwrappedIndex是没有wrap的bufferStartIndex=（0,0）情况下的坐标（cell）
    //即，unwrappedIndex是cell到内存坐标系原点（左上角，X正向下，Y正向右）的偏移（cell）
  return (getBufferOrderToMapFrameTransformation() *
          unwrappedIndex.matrix()).cast<double>();
    //返回值是内存中index到以内存左上角为原点，X正向上，Y正向左的坐标系的转换关系
}

inline Index getIndexFromIndexVector(
    const Vector& indexVector,
    const Size& bufferSize,
    const Index& bufferStartIndex)
{
    //将内存块（以左上角为原点，X正向上，Y正向左，坐标轴方向与world系相同）
    //内坐标系(cell)做180度旋转变换，变换后，内存块可以按图像坐标系理解，即X正向下，Y正向右。
    //此时，index表示了从world系（position，m）到内存（bufferIndex，cell）的转换
  Index index = (getMapFrameToBufferOrderTransformation() *
                indexVector.cast<int>()).array();
  return getBufferIndexFromIndex(index, bufferSize, bufferStartIndex);
}

inline BufferRegion::Quadrant getQuadrant(const Index& index,
                                          const Index& bufferStartIndex)
{
  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1])
    return BufferRegion::Quadrant::TopLeft;
  if (index[0] >= bufferStartIndex[0] && index[1] <  bufferStartIndex[1])
    return BufferRegion::Quadrant::TopRight;
  if (index[0] <  bufferStartIndex[0] && index[1] >= bufferStartIndex[1])
    return BufferRegion::Quadrant::BottomLeft;
  if (index[0] <  bufferStartIndex[0] && index[1] <  bufferStartIndex[1])
    return BufferRegion::Quadrant::BottomRight;
  return BufferRegion::Quadrant::Undefined;
}

bool getPositionFromIndex(Position& position,
                          const Index& index,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const double& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex)
{
    //既然index是和bufferSize作比较的，说明正常应该 index < bufferSize，
    //即index是以当前内存块的左上角（起始位置）开始计数的，是wrap后的
  if (!checkIfIndexInRange(index, bufferSize)) return false;
  Vector offset;


  getVectorToFirstCell(offset, mapLength, resolution);


  //offset是从map center到左上角的偏移(m)
  //getIndexVectorFromIndex()的返回值是index转换到以内存块左上角为原点，
  //X正向上，Y正向左的坐标系中的坐标（cell），上述坐标*分辨率+offset(m)，
  //转换到map frame坐标系(m)，再+mapPosition，转换到world系(m)
  position = mapPosition + offset + resolution *
             getIndexVectorFromIndex(index, bufferSize, bufferStartIndex);
  return true;
}

bool getIndexFromPosition(Index& index,
                          const Position& position,
                          const Length& mapLength,
                          const Position& mapPosition,
                          const double& resolution,
                          const Size& bufferSize,
                          const Index& bufferStartIndex)
{
  Vector offset;
  getVectorToOrigin(offset, mapLength);
    //position是点在全局世界坐标系下的坐标（m），world系与map frame可以重合
    //也可以不重合，但方向是一样的。
    //map frame的坐标原点就是map center，就是mapPosition（map center在world系下的坐标）
    //offset是从map frame到world系的偏移，即position=点在map frame下的坐标+offset
    //offset=mapPosition
    //position-mapPosition是把world系下的点转换到map frame下，保持坐标轴方向不变
    //在上一步基础上，（position-mapPosition）-offset是进一步转换到以map frame
    //左上角的点为原点的坐标系，保持坐标轴方向不变
    //故，indexVector描述了从world系下的点坐标(m)转换到内存块
    //（以左上角为原点，X正向上，Y正向左，坐标轴方向与world系相同）内坐标(cell)的转换
    //在getIndexFromIndexVector()中有对上述内存块坐标系的180度旋转变换，变换后，
    //内存块可以按图像坐标系理解，即X正向下，Y正向右。
  Vector indexVector = ((position - offset - mapPosition).array() /
                       resolution).matrix();
  index = getIndexFromIndexVector(indexVector, bufferSize, bufferStartIndex);
    //index是在内存块中wrap后的坐标(cell)，即index<bufferSize
  if (!checkIfPositionWithinMap(position, mapLength, mapPosition))
    return false;
  return true;
}

bool checkIfPositionWithinMap(const Position& position,
                              const Length& mapLength,
                              const Position& mapPosition)
{
  Vector offset;
  getVectorToOrigin(offset, mapLength);
  Position positionTransformed = getMapFrameToBufferOrderTransformation().cast<double>() *
                                 (position - mapPosition - offset);
  //positionTransformed是把world系下的点（m）转换到map frame左上角为原点，X正向下，Y正向右的坐标系中

  if (positionTransformed.x() >= 0.0 &&
      positionTransformed.y() >= 0.0 &&
      positionTransformed.x() < mapLength(0) &&
      positionTransformed.y() < mapLength(1)) {
    return true;
  }
  return false;
}

//@param[in] position: the position of the map.
void getPositionOfDataStructureOrigin(const Position& position,
                                      const Length& mapLength,
                                      Position& positionOfOrigin)
{
  //DataStructureOrigin是内存中存储的第一个cell，
  //对应的是map frame中左上角的position点
  //输入的position是map center，即mapPosition(m)
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);
  //vectorToOrigin是从map center到map中左上角点的偏移(m)
  positionOfOrigin = position + vectorToOrigin;
  //positionOfOrigin是在world系下(m)
}

bool getIndexShiftFromPositionShift(Index& indexShift,
                                    const Vector& positionShift,
                                    const double& resolution)
{
  Vector indexShiftVectorTemp = (positionShift.array() / resolution).matrix();
  Eigen::Vector2i indexShiftVector;

  for (int i = 0; i < indexShiftVector.size(); i++) {
    indexShiftVector[i] = static_cast<int>(indexShiftVectorTemp[i] +
                          0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
  //不懂为什么要在对应的方向上再+0.5？
  }
  //indexShiftVector与positionShift在同一个坐标系中，
  //而这个坐标系与indexShift所在的内存坐标系差180度
  indexShift = (getMapFrameToBufferOrderTransformation() *
               indexShiftVector).array();
  return true;
}

bool getPositionShiftFromIndexShift(Vector& positionShift,
                                    const Index& indexShift,
                                    const double& resolution)
{
    //坐标系差180度
  positionShift = (getBufferOrderToMapFrameTransformation() *
                  indexShift.matrix()).cast<double>() * resolution;
  return true;
}

//index和bufferSize都对应内存
bool checkIfIndexInRange(const Index& index, const Size& bufferSize)
{
  if (index[0] >= 0 && index[1] >= 0 &&
      index[0] < bufferSize[0] && index[1] < bufferSize[1])
  {
    return true;
  }
  return false;
}

void boundIndexToRange(Index& index, const Size& bufferSize)
{
  for (int i = 0; i < index.size(); i++) {
    boundIndexToRange(index[i], bufferSize[i]);
  }
}

void boundIndexToRange(int& index, const int& bufferSize)
{
  if (index < 0) index = 0;
  else if (index >= bufferSize) index = bufferSize - 1;
}

void wrapIndexToRange(Index& index, const Size& bufferSize)
{
  for (int i = 0; i < index.size(); i++) {
    wrapIndexToRange(index[i], bufferSize[i]);
  }
}

void wrapIndexToRange(int& index, const int& bufferSize)
{
  if (index < 0) index += ((-index / bufferSize) + 1) * bufferSize;
  //当坐标系以左上角为原点时，只有在bufferIndex(wrapped)->position(world)时，
  //本函数中的index才会是<0的
  //这个index表示wrapped后的cell与bufferStartIndex的偏移，
  //<0说明发生了wrap，>=0说明没有wrap
  index = index % bufferSize;
}

void boundPositionToRange(Position& position, const Length& mapLength,
                          const Position& mapPosition)
{
  Vector vectorToOrigin;
  getVectorToOrigin(vectorToOrigin, mapLength);
  Position positionShifted = position - mapPosition + vectorToOrigin;
    //positionShifted是从world系到以map右下角为原点，坐标轴方向不变的坐标系的转换

  // We have to make sure to stay inside the map.
  for (int i = 0; i < positionShifted.size(); i++) {
    // TODO Why is the factor 10 necessary.
    double epsilon = 10.0 * numeric_limits<double>::epsilon();
      //epsilon():最小非零浮点数
    if (std::fabs(position(i)) > 1.0) epsilon *= std::fabs(position(i));

    if (positionShifted(i) <= 0) {
      positionShifted(i) = epsilon;
      continue;
    }
    if (positionShifted(i) >= mapLength(i)) {
      positionShifted(i) = mapLength(i) - epsilon;
      continue;
    }
  }

  position = positionShifted + mapPosition - vectorToOrigin;
}

const Eigen::Matrix2i getBufferOrderToMapFrameAlignment()
{
  return getBufferOrderToMapFrameTransformation().array().abs().matrix();
}