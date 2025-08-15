# fine_nav2d_behavior

## 简介

`fine_nav2d_behavior` 是基于 ROS2 和 BehaviorTree.CPP 的自定义行为树插件包，提供了随机目标点生成功能，适用于机器人在指定区域内进行随机巡航或探索任务。

本包实现了一个名为 **`RandomGoalAction`** 的同步行为树节点，能够：

- 解析输入的多边形区域；
- 在该区域内部随机生成目标点；
- 将目标点发布至 `/goal_pose` 主题；
- 通过行为树输出端口返回该目标点。

---

## 依赖

本包基于以下 ROS2 及常规依赖：

- `rclcpp`
- `geometry_msgs`
- `tf2_geometry_msgs`
- `pluginlib`
- `behaviortree_cpp_v3`

---

## 编译

请确认已在 ROS2 工作空间下，执行以下命令：

```bash
colcon build --packages-select fine_nav2d_behavior
source install/setup.bash
```

## 行为树节点说明

### RandomGoalAction

| 端口类型 | 端口名       | 数据类型                          | 说明                               |
|----------|--------------|-----------------------------------|------------------------------------|
| 输入端口 | area         | std::string                       | 目标区域的字符串，格式为：[(x1,y1),(x2,y2),...] |
| 输出端口 | output_goal  | geometry_msgs::msg::PoseStamped   | 生成的随机目标位置                 |

#### 示例：输入区域格式

```xml
<RandomGoalAction area="[(0,0),(2,0),(2,2),(0,2)]" output_goal="{goal}" />
```

上述示例表示在矩形区域 (0,0) 到 (2,2) 内生成随机目标点。

#### 目标点发布

该节点在成功生成目标点后，会自动发布 geometry_msgs::msg::PoseStamped 消息到 /goal_pose 主题，便于系统中其他组件订阅该目标。

#### 插件注册

本包已通过 pluginlib 和 plugin.xml 正确导出插件，行为树工厂加载方式如下：

```cpp
BT::BehaviorTreeFactory factory;
factory.registerFromPlugin("libfine_nav2d_behavior.so");
```

或在 BehaviorTree XML 文件中直接引用：

```xml
<include path="$(find-pkg-share fine_nav2d_behavior)/behavior_trees/random_behavior_tree.xml" />
```

## 注意事项

- 输入的 area 参数需为封闭多边形点集；
- 当前随机点生成方式为简单矩形包围盒内随机，尚未做复杂多边形内点判定；
- 目标点默认发布在 "map" 坐标系下。



## 示例行为树文件 (behavior_trees/random_behavior_tree.xml)

```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="RootSequence">
      <RandomGoalAction area="[(0,0),(3,0),(3,3),(0,3)]" output_goal="{goal}" />
      <NavigateToPose goal="{goal}" />
```