#include "grid_map/grid_map.h"

GridMap::GridMap(ros::NodeHandle &nh): node(nh)
{
    /* get parameter */
    double x_size, y_size, z_size, vis_freq;
    node.param("grid_map/resolution", resolution_, 0.05);
    node.param("grid_map/map_size_x", x_size, 5.0);
    node.param("grid_map/map_size_y", y_size, 5.0);
    node.param("grid_map/map_size_z", z_size, 2.5);
    node.param("grid_map/uav_radius_inflation", radius_inflation_, 0.1);
    node.param("grid_map/extra_inflation", extra_inflation_, 0.08);
    node.param("grid_map/visualization_truncate_height", visualization_truncate_height_, 1.9);
    node.param("grid_map/virtual_ceil_height", virtual_ceil_height_, 2.0);
    node.param("grid_map/ground_height", ground_height_, -0.05);
    node.param("grid_map/world_frame_name", world_frame_id, string("world"));
    node.param("grid_map/visualization_frequency", vis_freq, 10.0);
    node.param("grid_map/local_sensing_range_xy", local_detect_range_xy, 2.0);
    node.param("grid_map/local_sensing_range_z", local_detect_range_z, 1.0);
    if(vis_freq <= 0){
        vis_freq = 10.0;
    }
    local_map_bound_(0) = ceil(local_detect_range_xy / resolution_);
    local_map_bound_(1) = local_map_bound_(0);
    local_map_bound_(2) = ceil(local_detect_range_z / resolution_);
    resolution_inv_ = 1 / resolution_;
    map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, ground_height_);
    map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    for (int i = 0; i < 3; ++i)
        map_voxel_num_(i) = ceil(map_size_(i) / resolution_);

    map_min_boundary_ = map_origin_;
    map_max_boundary_ = map_origin_ + map_size_;
    int buffer_size = map_voxel_num_(0) * map_voxel_num_(1) * map_voxel_num_(2);
    occupancy_buffer_ = vector<double>(buffer_size, 0.0);
    occupancy_buffer_inflate_ = vector<signed char>(buffer_size, 0);
    occupancy_buffer_inflate_extra_ = vector<signed char>(buffer_size, 0);
    has_global_map = false;

    global_cloud_sub_ =
        node.subscribe<sensor_msgs::PointCloud2>("/grid_map/cloud_in", 10, &GridMap::cloudCallback, this);
    uav_pose_sub_ =
        node.subscribe<geometry_msgs::PoseStamped>("/grid_map/uav_pose_in", 10, &GridMap::odomCallback, this);
    vis_timer_ = node.createTimer(ros::Duration(1.0/vis_freq), &GridMap::visCallback, this);
    local_map_pub_ = node.advertise<sensor_msgs::PointCloud2>("/grid_map/local_map", 10);
    local_map_inf_pub_ = node.advertise<sensor_msgs::PointCloud2>("/grid_map/local_map_inflate", 10);
}

void GridMap::reset()
{
    int buffer_size = map_voxel_num_(0) * map_voxel_num_(1) * map_voxel_num_(2);
    occupancy_buffer_ = vector<double>(buffer_size, 0.0);
    occupancy_buffer_inflate_ = vector<signed char>(buffer_size, 0);
    occupancy_buffer_inflate_extra_ = vector<signed char>(buffer_size, 0);

    cloud.clear();
    cloud_inflate.clear();
    cloud_inflate_extra.clear();
    has_global_map = false;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    /*Test only!*/
    std::cout << "Callback fcn called!" << std::endl;

    if(has_global_map)
        return;
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*msg, latest_cloud);
    if (latest_cloud.points.size() == 0)
        return;

    /*Test only!*/
    std::cout << "Received pcl of size " << latest_cloud.points.size() << std::endl;

    cloud = latest_cloud;

    int inf_step = ceil(radius_inflation_ / resolution_);
    int inf_step_extra = ceil((radius_inflation_ + extra_inflation_) / resolution_);
    int inf_step_z = 1;
    pcl::PointXYZ pt, pt_inf;
    Eigen::Vector3d p3d, p3d_inf;
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
        Eigen::Vector3i inf_pt;
        if (isInMap(p3d))
        {
            for (int x = -inf_step; x <= inf_step; ++x)
                for (int y = -inf_step; y <= inf_step; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z)
                    {
                        p3d_inf(0) = pt.x + x * resolution_;
                        p3d_inf(1) = pt.y + y * resolution_;
                        p3d_inf(2) = pt.z + z * resolution_;
                        posToIndex(p3d_inf, inf_pt);
                        if (!isInMap(inf_pt))
                            continue;
                        int idx_inf = toAddress(inf_pt);
                        occupancy_buffer_inflate_[idx_inf] = 1;

                        pt_inf.x = p3d_inf(0);
                        pt_inf.y = p3d_inf(1);
                        pt_inf.z = p3d_inf(2);
                        cloud_inflate.push_back(pt_inf);
                    }

            for (int x = -inf_step_extra; x <= inf_step_extra; ++x)
                for (int y = -inf_step_extra; y <= inf_step_extra; ++y)
                    for (int z = -inf_step_z; z <= inf_step_z; ++z)
                    {
                        p3d_inf(0) = pt.x + x * resolution_;
                        p3d_inf(1) = pt.y + y * resolution_;
                        p3d_inf(2) = pt.z + z * resolution_;
                        posToIndex(p3d_inf, inf_pt);
                        if (!isInMap(inf_pt))
                            continue;
                        int idx_inf = toAddress(inf_pt);
                        occupancy_buffer_inflate_extra_[idx_inf] = 1;

                        pt_inf.x = p3d_inf(0);
                        pt_inf.y = p3d_inf(1);
                        pt_inf.z = p3d_inf(2);
                        cloud_inflate_extra.push_back(pt_inf);
                    }
        }
    }
    this->simplifyPointClouds();
    has_global_map = true;
}

void GridMap::simplifyPointClouds(){
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setLeafSize(resolution_, resolution_, resolution_);
    pcl::PointCloud<pcl::PointXYZ> cloud_t, cloud_inflate_t, cloud_inflate_extra_t;
    cloud_t = cloud;
    cloud_inflate_t = cloud_inflate;
    cloud_inflate_extra_t = cloud_inflate_extra;

    sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_t));
    sor.filter(cloud);
    sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_inflate_t));
    sor.filter(cloud_inflate);
    sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_inflate_extra_t));
    sor.filter(cloud_inflate_extra);
}

void GridMap::odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    uav_pos(0) = msg->pose.position.x;
    uav_pos(1) = msg->pose.position.y;
    uav_pos(2) = msg->pose.position.z;
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
    if (local_map_pub_.getNumSubscribers() + local_map_inf_pub_.getNumSubscribers() <= 0 || !has_global_map)
        return;

    Eigen::Vector3i center;
    posToIndex(uav_pos, center);

    Eigen::Vector3d pt_pos;
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> local_cloud, local_cloud_inf;
    Eigen::Vector3i p_idx;
    for (int dx = -local_map_bound_(0); dx <= local_map_bound_(0); ++dx)
        for (int dy = -local_map_bound_(1); dy <= local_map_bound_(1); ++dy)
            for (int dz = -local_map_bound_(2); dz <= local_map_bound_(2); ++dz)
            {
                p_idx = Eigen::Vector3i(center(0)+dx, center(1)+dy, center(2)+dz);
                if(!isInMap(p_idx))
                    continue;
                if(isOccupied(p_idx)){
                    indexToPos(p_idx, pt_pos);
                    if (pt_pos(2) > visualization_truncate_height_)
                        continue;
                    pt.x = pt_pos(0);
                    pt.y = pt_pos(1);
                    pt.z = pt_pos(2);
                    local_cloud.push_back(pt);
                    local_cloud_inf.push_back(pt);
                }
                else if(isOccupiedInflate(p_idx)){
                    indexToPos(p_idx, pt_pos);
                    if (pt_pos(2) > visualization_truncate_height_)
                        continue;
                    pt.x = pt_pos(0);
                    pt.y = pt_pos(1);
                    pt.z = pt_pos(2);
                    local_cloud_inf.push_back(pt);
                }
            }

    local_cloud.width = local_cloud.points.size();
    local_cloud.height = 1;
    local_cloud.is_dense = true;
    local_cloud.header.frame_id = world_frame_id;
    sensor_msgs::PointCloud2 local_cloud_msg;
    pcl::toROSMsg(local_cloud, local_cloud_msg);
    local_map_pub_.publish(local_cloud_msg);

    local_cloud_inf.width = local_cloud_inf.points.size();
    local_cloud_inf.height = 1;
    local_cloud_inf.is_dense = true;
    local_cloud_inf.header.frame_id = world_frame_id;
    sensor_msgs::PointCloud2 local_cloud_inf_msg;
    pcl::toROSMsg(local_cloud_inf, local_cloud_inf_msg);
    local_map_inf_pub_.publish(local_cloud_inf_msg);
}