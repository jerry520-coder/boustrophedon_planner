#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "boustrophedon_server/boustrophedon_planner_server.h"

BoustrophedonPlannerServer::BoustrophedonPlannerServer()
  : private_node_handle_("~") // 初始化私有节点句柄
  , action_server_(node_handle_, "plan_path", boost::bind(&BoustrophedonPlannerServer::executePlanPathAction, this, _1),// 初始化行动服务器
                   false)
  , conversion_server_{ node_handle_.advertiseService("convert_striping_plan_to_path",
                                                      &BoustrophedonPlannerServer::convertStripingPlanToPath, this) } // 初始化服务服务器
{
  std::size_t error = 0; // 记录参数获取错误的数量

   // 获取 ROS 参数，如果获取失败则增加错误计数
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "repeat_boundary", repeat_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_clockwise", outline_clockwise_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "skip_outlines", skip_outlines_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "outline_layer_count", outline_layer_count_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_separation", stripe_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "intermediary_separation", intermediary_separation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "stripe_angle", stripe_angle_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get("plan_path", private_node_handle_,
                                                             "enable_stripe_angle_orientation", enable_orientation_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "travel_along_boundary", travel_along_boundary_));
  error += static_cast<std::size_t>(!rosparam_shortcuts::get(
      "plan_path", private_node_handle_, "allow_points_outside_boundary", allow_points_outside_boundary_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "enable_half_y_turns", enable_half_y_turns_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "points_per_turn", points_per_turn_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "turn_start_offset", turn_start_offset_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_polygons", publish_polygons_));
  error += static_cast<std::size_t>(
      !rosparam_shortcuts::get("plan_path", private_node_handle_, "publish_path_points", publish_path_points_));
  rosparam_shortcuts::shutdownIfError("plan_path", error);


// 检查 intermediary_separation 参数是否合法
  if (intermediary_separation_ <= 0.0)
  {
    // doesn't make sense, or we don't want intermediaries. set it to double max so we can't make any intermediaries
    // 如果不合法，则将其设置为最大值，这样就不会生成中间路径
    intermediary_separation_ = std::numeric_limits<double>::max();
  }

 // 检查 enable_half_y_turns 参数与 outline_layer_count 参数是否冲突
  if (enable_half_y_turns_ && outline_layer_count_ < 1)
  {
    if (allow_points_outside_boundary_)
    {
      ROS_WARN_STREAM("Current configuration will result in turns that go outside the boundary, but this has been "
                      "explicitly enabled"); //当前配置会导致转弯超出边界，但这种情况已被显式允许/-
    }
    else
    {
      // we can't do half-y-turns safely without an inner boundary layer, as the arc will stick outside of the boundary
        // 如果不允许点超出边界且没有内层边界层，则无法安全地进行半Y转弯
      ROS_ERROR_STREAM("Cannot plan using half-y-turns if the outline_layer_count is less than 1! Boustrophedon "
                       "planner will not start.");
      return;
    }
  }

 // 设置 striping_planner_ 的参数
  striping_planner_.setParameters({ stripe_separation_, intermediary_separation_, travel_along_boundary_,
                                    enable_half_y_turns_, points_per_turn_, turn_start_offset_ });

  // 设置 outline_planner_ 的参数
  outline_planner_.setParameters(
      { repeat_boundary_, outline_clockwise_, skip_outlines_, outline_layer_count_, stripe_separation_ });

  action_server_.start();// 启动行动服务器


 // 如果需要发布多边形数据，则初始化相应的发布者
  if (publish_polygons_)
  {
    initial_polygon_publisher_ = private_node_handle_.advertise<geometry_msgs::PolygonStamped>("initial_polygon", 1);
    preprocessed_polygon_publisher_ =
        private_node_handle_.advertise<geometry_msgs::PolygonStamped>("preprocessed_polygon", 1);
  }

  // mainly for use with plotJuggler, which wants the points to be put one at a time on the same topic
   // 如果需要发布路径点数据，则初始化相应的发布者
  if (publish_path_points_)
  {
    path_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("path_points", 1000);
    polygon_points_publisher_ = private_node_handle_.advertise<geometry_msgs::PointStamped>("polygon_points", 1000);
  }
}

void BoustrophedonPlannerServer::executePlanPathAction(const boustrophedon_msgs::PlanMowingPathGoalConstPtr& goal)
{
  std::string boundary_frame = goal->property.header.frame_id; // 从目标消息中获取边界帧ID

   // 如果启用了方向，则从机器人的当前位置获取条纹角度
  if (enable_orientation_)
  {
    stripe_angle_ = getStripeAngleFromOrientation(goal->robot_position);
  }

  Polygon polygon = fromBoundary(goal->property); // 从目标消息中获取多边形边界

  // 检查多边形是否合法（至少有三个顶点）
  if (!checkPolygonIsValid(polygon))
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner does not work for polygons of "
                                                            "size "
                                                            "< 3."));
    return;
  }

  // 检查多边形是否为简单多边形（非自相交）
  if (!polygon.is_simple())
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner only works for simple (non "
                                                            "self-intersecting) polygons."));
    return;
  }

    // 定义机器人位置
  Point robot_position;
  try
  {
    // 将机器人位置从带有帧ID的位置转换为无帧ID的位置
    robot_position = fromPositionWithFrame(goal->robot_position, goal->property.header.frame_id);
  }
  catch (const tf::TransformException& ex)
  {
    action_server_.setAborted(Server::Result(),
                              std::string("Boustrophedon planner failed with a tf exception: ") + ex.what());
    return;
  }
  std::vector<NavPoint> path;  // 定义路径向量

 // 预处理多边形
  auto preprocess_transform = preprocessPolygon(polygon, robot_position, stripe_angle_);


 // 如果需要发布多边形，则发布初始多边形和预处理后的多边形
  if (publish_polygons_)
  {
    initial_polygon_publisher_.publish(goal->property);
    preprocessed_polygon_publisher_.publish(convertCGALPolygonToMsg(polygon));
  }

  Polygon fill_polygon; // 定义填充多边形

  // 尝试将边界添加到路径中
  if (!outline_planner_.addToPath(polygon, robot_position, path, fill_polygon))
  {
    action_server_.setAborted(Server::Result(), std::string("Boustrophedon planner failed because "
                                                            "outline_layer_count "
                                                            "was too large for the boundary."));
    return;
  }

   // 定义多边形分解器
  PolygonDecomposer polygon_decomposer{};

  // A print statement MUST be here, see issue #1586
  ROS_INFO_STREAM("Decomposing boundary polygon into sub-polygons...");

   // 分解填充多边形
  polygon_decomposer.decompose(fill_polygon);
  std::vector<Polygon> sub_polygons = polygon_decomposer.getSubPolygons(robot_position); // 获取子多边形列表

  ROS_INFO_STREAM("Broke the boundary up into " << sub_polygons.size() << " sub-polygons");

  // 定义合并多边形和起始位置
  Polygon merged_polygon;
  Point start_position = robot_position;

   // 遍历子多边形
  for (const auto& subpoly : sub_polygons)
  {
    // combine the new subpoly with the merged_polygon. If merged_polygon is empty, it returns the sub polygon
     // 将新的子多边形与合并多边形组合。如果合并多边形为空，则返回子多边形
    merged_polygon = mergePolygons(merged_polygon, subpoly);

    // add the stripes to the path, using merged_polygon boundary to travel if necessary.
    // 使用合并多边形的边界添加条纹到路径中
    striping_planner_.addToPath(merged_polygon, subpoly, robot_position, path);
  }

  // 如果需要沿边界旅行，则添加返回起点的路径
  if (travel_along_boundary_)
  {
    striping_planner_.addReturnToStart(merged_polygon, start_position, robot_position, path);
  }

   // 后处理多边形和路径
  postprocessPolygonAndPath(preprocess_transform, polygon, path);

  // 如果需要可视化规划路径，则发布路径点和多边形点
  if (publish_path_points_)  // if we care about visualizing the planned path in plotJuggler
  {
    publishPathPoints(path);
    publishPolygonPoints(polygon);
  }

  // 将路径转换为结果并设置成功状态
  auto result = toResult(std::move(path), boundary_frame);
  action_server_.setSucceeded(result);
}

boustrophedon_msgs::PlanMowingPathResult BoustrophedonPlannerServer::toResult(std::vector<NavPoint>&& path,
                                                                         const std::string& frame) const
{
  boustrophedon_msgs::PlanMowingPathResult result;
  result.plan.points.reserve(path.size()); // 预留空间以提高性能
  result.plan.header.stamp = ros::Time::now();  // 设置消息的时间戳
  result.plan.header.frame_id = frame; // 设置消息的帧ID

 // 遍历路径中的每个点，将其转换为StripingPoint类型并添加到结果中
  for (const auto& point : path)
  {
    boustrophedon_msgs::StripingPoint stripe_point{};
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    stripe_point.point.z = 0;
    stripe_point.type = static_cast<uint8_t>(point.type);// 设置点的类型
    result.plan.points.emplace_back(stripe_point);
  }
  return result;
}

Polygon BoustrophedonPlannerServer::fromBoundary(const geometry_msgs::PolygonStamped& boundary) const
{
  Polygon polygon;

   // 遍历边界中的每个点，并将其添加到多边形中
  for (const auto& point : boundary.polygon.points)
  {
    polygon.push_back(Point(point.x, point.y));
  }
  return polygon;
}

Point BoustrophedonPlannerServer::fromPositionWithFrame(const geometry_msgs::PoseStamped& pose,
                                                        const std::string& target_frame) const
{
  geometry_msgs::PoseStamped transformed_pose;

   // 使用tf监听器将位置转换到目标帧
  transform_listener_.transformPose(target_frame, pose, transformed_pose);
  return { transformed_pose.pose.position.x, transformed_pose.pose.position.y };
}

// 将条纹规划转换为路径
bool BoustrophedonPlannerServer::convertStripingPlanToPath(boustrophedon_msgs::ConvertPlanToPath::Request& request,
                                                           boustrophedon_msgs::ConvertPlanToPath::Response& response)
{
  response.path.header.frame_id = request.plan.header.frame_id; // 设置路径的帧ID
  response.path.header.stamp = request.plan.header.stamp; // 设置路径的时间戳

 // 将每个条纹点转换为路径中的姿态
  std::transform(request.plan.points.begin(), request.plan.points.end(), response.path.poses.begin(),
                 [&](const boustrophedon_msgs::StripingPoint& point) {
                   geometry_msgs::PoseStamped pose;
                   pose.header.frame_id = request.plan.header.frame_id;
                   pose.header.stamp = request.plan.header.stamp;
                   pose.pose.position = point.point;
                   pose.pose.orientation.x = 0.0;
                   pose.pose.orientation.y = 0.0;
                   pose.pose.orientation.z = 0.0;
                   pose.pose.orientation.w = 1.0;
                   return pose;
                 });
  return true;
}


// 将CGAL多边形转换为ROS消息
geometry_msgs::PolygonStamped BoustrophedonPlannerServer::convertCGALPolygonToMsg(const Polygon& poly) const
{
  geometry_msgs::PolygonStamped stamped_poly;

 // 遍历多边形的每个顶点，并将其添加到消息中
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::Point32 point;
    point.x = float(it->x());
    point.y = float(it->y());
    point.z = float(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = ros::Time::now();
  return stamped_poly;
}

// 检查多边形是否合法（顶点数是否不少于3个）
bool BoustrophedonPlannerServer::checkPolygonIsValid(const Polygon& poly) const
{
  return !(poly.size() < 3);  // expand later if we find more cases of invalid polygons
}

// get the yaw from the robot_position part of the given goal
// 从给定目标的 robot_position 部分获取偏航角（yaw）
double BoustrophedonPlannerServer::getStripeAngleFromOrientation(const geometry_msgs::PoseStamped& robot_position)
{
  tf::Quaternion quat(robot_position.pose.orientation.x, robot_position.pose.orientation.y,
                      robot_position.pose.orientation.z, robot_position.pose.orientation.w);

  // 将四元数转换为旋转矩阵
  tf::Matrix3x3 m(quat);
  double roll, pitch, yaw;

   // 从旋转矩阵中提取滚转角（roll）、俯仰角（pitch）和偏航角（yaw）
  m.getRPY(roll, pitch, yaw);

  ROS_INFO_STREAM("Got Striping Angle from Orientation: " << yaw);
  // TODO: Recalibrate the IMU so that we can get rid of this constant below.
   // TODO: 重新校准IMU以去除下面的常数
  return yaw + 1.57079632679;  // Adds PI / 2 to account for incorrect IMU calibration / reference vector // 添加 PI / 2 以补偿IMU校准/参考向量的错误
}

// publishes the path points one at a time for visualization in plotJuggler
// 逐个发布路径点以在 plotJuggler 中进行可视化
void BoustrophedonPlannerServer::publishPathPoints(const std::vector<NavPoint>& path) const
{
  for (NavPoint point : path)
  {
    geometry_msgs::PointStamped stripe_point{};
    stripe_point.header.stamp = ros::Time::now();
    stripe_point.point.x = point.point.x();
    stripe_point.point.y = point.point.y();
    path_points_publisher_.publish(stripe_point);
    ros::spinOnce();// 处理一次回调，以确保消息发布
  }
}

// publishes the polygon points one at a time for visualization in plotJuggler
// 逐个发布多边形点以在 plotJuggler 中进行可视化
void BoustrophedonPlannerServer::publishPolygonPoints(const Polygon& poly) const
{
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++)
  {
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.point.x = float(it->x());
    point.point.y = float(it->y());
    point.point.z = float(0.0);
    polygon_points_publisher_.publish(point);
    ros::spinOnce(); // 处理一次回调，以确保消息发布
  }
}