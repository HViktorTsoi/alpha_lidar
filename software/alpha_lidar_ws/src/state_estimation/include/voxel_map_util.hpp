#ifndef VOXEL_MAP_UTIL_HPP
#define VOXEL_MAP_UTIL_HPP
#include "common_lib.h"
#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
//#include <execution>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define HASH_P 116101
#define MAX_N 10000000000

static int plane_id = 0;

state_ikfom current_state_point;

// a point to plane matching structure
typedef struct ptpl {
  // Must enable for some devices!!!!!!!
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point;
  Eigen::Vector3d point_world;
  Eigen::Vector3d normal;
  Eigen::Vector3d center;
  Eigen::Matrix<double, 6, 6> plane_cov;
  double d;
  int layer;
  Eigen::Matrix3d cov_lidar;
} ptpl;

// 3D point with covariance
typedef struct pointWithCov {
  // Must enable for some devices!!!!!!!
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d point;
  Eigen::Vector3d point_world;
  Eigen::Matrix3d cov;
  Eigen::Matrix3d cov_lidar;
} pointWithCov;

typedef struct Plane {
  // Must enable for some devices!!!!!!!
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  Eigen::Vector3d y_normal;
  Eigen::Vector3d x_normal;
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 6, 6> plane_cov;
  float radius = 0;
  float min_eigen_value = 1;
  float mid_eigen_value = 1;
  float max_eigen_value = 1;
  float d = 0;
  int points_size = 0;

  bool is_plane = false;
  bool is_init = false;
  int id;
  // is_update and last_update_points_size are only for publish plane
  bool is_update = false;
  int last_update_points_size = 0;
  bool update_enable = true;

  // 计算平面与视点的夹角cos值
  double calc_normal_viewpoint_cos(const Eigen::Vector3d& viewpoint) {
      // 注意 这里应该是平面中心与当前帧位置的距离;
      //  而且注意向量的方向，想求的是(法线中心-法线末端，法线中心-原点)这两个向量的夹角
      Eigen::Vector3d point_center = viewpoint - center;
      double cos_val = point_center.dot(normal) / (point_center.norm() * normal.norm()); //角度cos值
      return cos_val;
  }

  // 更新平面参数
  void update_parameter(const Eigen::Matrix3cd& evecs,
                        const Eigen::Vector3d& evalsReal,
                        const Eigen::Matrix3f::Index& evalsMin,
                        const Eigen::Matrix3f::Index& evalsMid,
                        const Eigen::Matrix3f::Index& evalsMax){
      // 估计法线方向
      normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
              evecs.real()(2, evalsMin);

//      // TODO 目前每次调用update_parameter都重新整理法线方向， 即用最新的一帧来更新法线方向，后续考虑用平面所有的历史观测中较多的方向来去掉outlier
//      // 计算normal与点-点夹角
//      double cos_val = calc_normal_viewpoint_cos(current_state_point.pos);
//      // 判断是锐角还是钝角 如果是钝角, 则将法线反向， 保证法线是朝向LiDAR的
//      if(cos_val < 0) {
//          normal = -normal;
//      }

      y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
              evecs.real()(2, evalsMid);
      x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
              evecs.real()(2, evalsMax);
      min_eigen_value = evalsReal(evalsMin);
      mid_eigen_value = evalsReal(evalsMid);
      max_eigen_value = evalsReal(evalsMax);
      radius = sqrt(evalsReal(evalsMax));
      d = -(normal(0) * center(0) +
            normal(1) * center(1) +
            normal(2) * center(2));
  }
} Plane;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
} // namespace std

class OctoTree {
public:
  // Must enable for some devices!!!!!!!
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<pointWithCov> temp_points_; // all points in an octo tree
  // std::vector<pointWithCov> temp_points_2_; // 第二组法线相反的点
  std::vector<pointWithCov> new_points_;  // new points in an octo tree
  Plane *plane_ptr_;
  // Plane *plane_ptr_2_; // 第二组平面
  int max_layer_;
  bool indoor_mode_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_point_size_;
  float quater_length_;
  float planer_threshold_;
  int max_plane_update_threshold_;
  int update_size_threshold_;
  int all_points_num_;
  int new_points_num_;
  int max_points_size_;
  int max_cov_points_size_;
  bool init_octo_;
  bool update_cov_enable_;
  bool update_enable_;

  // 每个voxel有自己的color用于可视化
  std::vector<unsigned int> colors;

  OctoTree(int max_layer, int layer, std::vector<int> layer_point_size,
           int max_point_size, int max_cov_points_size, float planer_threshold)
      : max_layer_(max_layer), layer_(layer),
        layer_point_size_(layer_point_size), max_points_size_(max_point_size),
        max_cov_points_size_(max_cov_points_size),
        planer_threshold_(planer_threshold) {
    temp_points_.clear();
    // temp_points_2_.clear();
    octo_state_ = 0;
    new_points_num_ = 0;
    all_points_num_ = 0;
    // when new points num > 5, do a update
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    update_cov_enable_ = true;
    max_plane_update_threshold_ = layer_point_size_[layer_];
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
    // plane_ptr_2_ = new Plane;

    colors.push_back(static_cast<unsigned int>(rand() % 256));
    colors.push_back(static_cast<unsigned int>(rand() % 256));
    colors.push_back(static_cast<unsigned int>(rand() % 256));
  }

  // check is plane , calc plane parameters including plane covariance
  void init_plane(const std::vector<pointWithCov> &points, Plane *plane) {
    // TODO 每个voxel拆分成2个plane 这需要看temp points的结构
    // TODO 每次state estimation之后，考虑更新点到平面的关联关系，用点面法线夹角将点划分为两类（正面和反面），后续分别根据正反面来更新对应的平面
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = points.size();
    plane->radius = 0;
    for (auto pv : points) {
      plane->covariance += pv.point * pv.point.transpose();
      plane->center += pv.point;
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size -
                        plane->center * plane->center.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    // plane covariance calculation
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
        1.0 / plane->points_size;
    if (evalsReal(evalsMin) < planer_threshold_) {
      std::vector<int> index(points.size());
      std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> temp_matrix(points.size());
      for (int i = 0; i < points.size(); i++) {
        Eigen::Matrix<double, 6, 3> J;
        Eigen::Matrix3d F;
        for (int m = 0; m < 3; m++) {
          if (m != (int)evalsMin) {
            Eigen::Matrix<double, 1, 3> F_m =
                (points[i].point - plane->center).transpose() /
                ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                 evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
            F.row(m) = F_m;
          } else {
            Eigen::Matrix<double, 1, 3> F_m;
            F_m << 0, 0, 0;
            F.row(m) = F_m;
          }
        }
        J.block<3, 3>(0, 0) = evecs.real() * F;
        J.block<3, 3>(3, 0) = J_Q;
        plane->plane_cov += J * points[i].cov * J.transpose();
      }
      // 更新平面参数
      plane->update_parameter(evecs, evalsReal, evalsMin, evalsMid, evalsMax);
      plane->is_plane = true;
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }

      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }

    } else {
      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }
      plane->is_plane = false;
      // 更新平面参数
      plane->update_parameter(evecs, evalsReal, evalsMin, evalsMid, evalsMax);
    }
  }

  // only updaye plane normal, center and radius with new points
  // 只更新平面的参数，不更新平面的cov
  void update_plane(const std::vector<pointWithCov> &points, Plane *plane) {
    Eigen::Matrix3d old_covariance = plane->covariance;
    Eigen::Vector3d old_center = plane->center;
    Eigen::Matrix3d sum_ppt =
        (plane->covariance + plane->center * plane->center.transpose()) *
        plane->points_size;
    Eigen::Vector3d sum_p = plane->center * plane->points_size;
    for (size_t i = 0; i < points.size(); i++) {
      Eigen::Vector3d pv = points[i].point;
      sum_ppt += pv * pv.transpose();
      sum_p += pv;
    }
    plane->points_size = plane->points_size + points.size();
    plane->center = sum_p / plane->points_size;
    plane->covariance = sum_ppt / plane->points_size -
                        plane->center * plane->center.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    if (evalsReal(evalsMin) < planer_threshold_) {
      // 更新平面参数
      plane->update_parameter(evecs, evalsReal, evalsMin, evalsMid, evalsMax);
      plane->is_plane = true;
      plane->is_update = true;
    } else {
      // 更新平面参数
      plane->update_parameter(evecs, evalsReal, evalsMin, evalsMid, evalsMax);
      plane->is_plane = false;
      plane->is_update = true;
    }
  }

  void init_octo_tree() {
    // 如果临时点数量大于更新阈值(很小的数值，大概是5个点左右)
    if (temp_points_.size() > max_plane_update_threshold_) {
      // 检测所有临时点是否为平面，如果是的话计算平面参数
      init_plane(temp_points_, plane_ptr_);
      if (plane_ptr_->is_plane == true) {
        // 如果是平面，直接作为叶节点
        octo_state_ = 0;
        // 判断存储的点是否过多，如果过多并且超过阈值就停止更新cov和平面参数
        if (temp_points_.size() > max_cov_points_size_) {
          update_cov_enable_ = false;
        }
        if (temp_points_.size() > max_points_size_) {
          update_enable_ = false;
        }
      } else {
        // 如果不是平面，继续划分voxel，并将当前节点标记为非叶节点
        octo_state_ = 1;
        cut_octo_tree();
      }
      // 完成构建，标记
      init_octo_ = true;
      new_points_num_ = 0;
      //      temp_points_.clear();
    }
  }

  void cut_octo_tree() {
    // 如果当前层数大于最大层数，直接返回，将当前节点标记为叶节点
    if (layer_ >= max_layer_) {
      octo_state_ = 0;
      return;
    }

    // 划分八叉树
    // 遍历当前节点的所有临时点 划分到对应的象限中
    for (size_t i = 0; i < temp_points_.size(); i++) {
      // 判断当前临时点属于哪个象限
      int xyz[3] = {0, 0, 0};
      if (temp_points_[i].point[0] > voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (temp_points_[i].point[1] > voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (temp_points_[i].point[2] > voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves_[leafnum] == nullptr) {
        leaves_[leafnum] = new OctoTree(
            max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
            max_cov_points_size_, planer_threshold_);
        // 算出来的position实际上是voxel的左下角，中心点需要在原中心点的基础上加半个voxel的偏移
        // (2 * xyz[0] - 1)的结果是-1(如果在父中心点左侧)或者1(如果在父中心点右侧)
        // 注意在第一层时，quater是voxel_size/4, 所以这里(2 * xyz[0] - 1) * quater_length在第一层就是0.5个子voxel的大小
        // 在其他层时， (2 * xyz[0] - 1) * quater_length为当前子voxel_size/2
        leaves_[leafnum]->voxel_center_[0] =
            voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[1] =
            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[2] =
            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      }
      leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
      leaves_[leafnum]->new_points_num_++;
    }
    // 对于划分好的每个子voxel中的临时点，检测是否构成为平面，如果是的话计算平面参数
    for (uint i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        if (leaves_[i]->temp_points_.size() >
            leaves_[i]->max_plane_update_threshold_) {
          init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
          // 如果是平面，那么设置为叶子节点，否则继续嵌套划分voxel
          if (leaves_[i]->plane_ptr_->is_plane) {
            leaves_[i]->octo_state_ = 0;
          } else {
            leaves_[i]->octo_state_ = 1;
            leaves_[i]->cut_octo_tree();
          }
          // 完成构建 标记
          leaves_[i]->init_octo_ = true;
          leaves_[i]->new_points_num_ = 0;
        }
      }
    }
  }

  void UpdateOctoTree(const pointWithCov &pv) {
    // 如果当前voxel中的点数量过少，还没有初始化voxel
    if (!init_octo_) {
      new_points_num_++;
      all_points_num_++;
      temp_points_.push_back(pv);
      // 如果点数增加到大于阈值，就初始化当前voxel
      if (temp_points_.size() > max_plane_update_threshold_) {
        init_octo_tree();
      }
    } else {
      // 如果当前voxel是平面(历史地图中的信息)
      // TODO 双平面的情况 需要判断当前两个平面是否有一个为平面
      if (plane_ptr_->is_plane) {
        // 如果点数未超过最大收敛要求点数，可以更新平面参数
        if (update_enable_) {
          // 计算当前视线到第一平面法线的夹角
          double cos_normal_viewvector = plane_ptr_->calc_normal_viewpoint_cos(current_state_point.pos);
          // new_points_num_和 all_points_num_是同一个量，只不过new_points_num_是滚动计数
          new_points_num_++;
          all_points_num_++;
          if (update_cov_enable_) {
            temp_points_.push_back(pv);
//            // TODO 在更新temp points时，如果是钝角就存到第二组temp points中，更新平面是用同样的init_plane, 参数换成第二组plane_ptr_和temp points
//            if (cos_normal_viewvector > 0){
//                temp_points_.push_back(pv);
//            }
//            else{
//                temp_points_2_.push_back(pv);
//            }
          } else {
            new_points_.push_back(pv);
          }
          // 每当新增加的点超过5个 更新平面参数
          // 这里等价于 if( new_points_num_ % 5 == 0 )
          // 更新的方式是重新估计平面参数和cov
          if (new_points_num_ > update_size_threshold_) {
            if (update_cov_enable_) {
              init_plane(temp_points_, plane_ptr_);
//              // TODO 判断当前点与voxel中的plane_ptr_成锐角还是钝角， 如果是钝角，更新第二组平面
//              if( cos_normal_viewvector > 0) {
//                  init_plane(temp_points_, plane_ptr_);
//              }
//              else{
//                  // 如果第一组平面与当前视点法线夹角为钝角，则说明当前视点与这个平面特征不应该建立关联关系， 而是应该去更新第二组点
//                  if(temp_points_2_.size() > update_size_threshold_){
//                    init_plane(temp_points_2_, plane_ptr_2_);
//                  }
//              }
            }
            new_points_num_ = 0;
          }
          // 如果总点数超过最大cov收敛要求点数，禁止更新cov，并且清空临时点
          if (all_points_num_ >= max_cov_points_size_) {
            update_cov_enable_ = false;
            std::vector<pointWithCov>().swap(temp_points_);
          }
          // 如果总点数超过最大平面收敛要求点数，禁止更新平面，并且清空新增点
          if (all_points_num_ >= max_points_size_) {
            update_enable_ = false;
            plane_ptr_->update_enable = false;
            std::vector<pointWithCov>().swap(new_points_);
          }
        } else {
          return;
        }
      } else {
        // 如果当前voxel不是平面
        if (layer_ < max_layer_) {
          // 如果当前为非叶节点
          // TODO (为什么？ )清空临时点 (猜测可能是需要处理原本是平面，增加点之后不是平面的情况)
          if (temp_points_.size() != 0) {
            std::vector<pointWithCov>().swap(temp_points_);
          }
          if (new_points_.size() != 0) {
            std::vector<pointWithCov>().swap(new_points_);
          }
          // 继续向下查找当前点所在的voxel
          int xyz[3] = {0, 0, 0};
          if (pv.point[0] > voxel_center_[0]) {
            xyz[0] = 1;
          }
          if (pv.point[1] > voxel_center_[1]) {
            xyz[1] = 1;
          }
          if (pv.point[2] > voxel_center_[2]) {
            xyz[2] = 1;
          }
          int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
          // 更新或者新建叶子节点
          if (leaves_[leafnum] != nullptr) {
            leaves_[leafnum]->UpdateOctoTree(pv);
          } else {
            leaves_[leafnum] = new OctoTree(
                max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                max_cov_points_size_, planer_threshold_);
            leaves_[leafnum]->layer_point_size_ = layer_point_size_;
            leaves_[leafnum]->voxel_center_[0] =
                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] =
                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] =
                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            leaves_[leafnum]->UpdateOctoTree(pv);
          }
        } else {
          // 如果当前节点为叶节点
          if (update_enable_) {
            // 将当前点加入到临时点中
            new_points_num_++;
            all_points_num_++;
            if (update_cov_enable_) {
              temp_points_.push_back(pv);
            } else {
              new_points_.push_back(pv);
            }
            // 每增加5个新点，更新一次平面
            if (new_points_num_ > update_size_threshold_) {
              // 如果点数低于最大限制，同时更新平面参数和cov
              if (update_cov_enable_) {
                init_plane(temp_points_, plane_ptr_);
              } else {
                // 否则只更新平面参数, 注意更新完清空new_points, 这里是用最新的5个点加上历史平面参数来计算新的平面参数
                update_plane(new_points_, plane_ptr_);
                new_points_.clear();
              }
              new_points_num_ = 0;
            }
            // 如果点数多于设定阈值，清空临时点
            if (all_points_num_ >= max_cov_points_size_) {
              update_cov_enable_ = false;
              std::vector<pointWithCov>().swap(temp_points_);
            }
            if (all_points_num_ >= max_points_size_) {
              update_enable_ = false;
              plane_ptr_->update_enable = false;
              std::vector<pointWithCov>().swap(new_points_);
            }
          }
        }
      }
    }
  }
};

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void buildVoxelMap(const std::vector<pointWithCov> &input_points,
                   const float voxel_size, const int max_layer,
                   const std::vector<int> &layer_point_size,
                   const int max_points_size, const int max_cov_points_size,
                   const float planer_threshold,
                   std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {
  uint plsize = input_points.size();
  for (uint i = 0; i < plsize; i++) {
    const pointWithCov p_v = input_points[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_v.point[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      feat_map[position]->temp_points_.push_back(p_v);
      feat_map[position]->new_points_num_++;
    } else {
      OctoTree *octo_tree =
          new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                       max_cov_points_size, planer_threshold);
      // 算出来的position实际上是voxel的左下角，中心点需要加0.5
      // TODO 确定为什么这里voxel_size要除以4
      feat_map[position] = octo_tree;
      feat_map[position]->quater_length_ = voxel_size / 4;
      feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      feat_map[position]->temp_points_.push_back(p_v);
      feat_map[position]->new_points_num_++;
      feat_map[position]->layer_point_size_ = layer_point_size;
    }
  }
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
    iter->second->init_octo_tree();
  }
}

//void updateVoxelMap(const std::vector<pointWithCov> &input_points,
//                    const float voxel_size, const int max_layer,
//                    const std::vector<int> &layer_point_size,
//                    const int max_points_size, const int max_cov_points_size,
//                    const float planer_threshold,
//                    std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {
//    uint plsize = input_points.size();
//    for (uint i = 0; i < plsize; i++) {
//        const pointWithCov p_v = input_points[i];
//        float loc_xyz[3];
//        for (int j = 0; j < 3; j++) {
//            loc_xyz[j] = p_v.point[j] / voxel_size;
//            if (loc_xyz[j] < 0) {
//                loc_xyz[j] -= 1.0;
//            }
//        }
//        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                           (int64_t)loc_xyz[2]);
//        auto iter = feat_map.find(position);
//        // 如果点的位置已经存在voxel 那么就更新点的位置 否则创建新的voxel
//        if (iter != feat_map.end()) {
//            feat_map[position]->UpdateOctoTree(p_v);
//        } else {
//            OctoTree *octo_tree =
//                    new OctoTree(max_layer, 0, layer_point_size, max_points_size,
//                                 max_cov_points_size, planer_threshold);
//            feat_map[position] = octo_tree;
//            feat_map[position]->quater_length_ = voxel_size / 4;
//            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
//            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
//            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
//            feat_map[position]->UpdateOctoTree(p_v);
//        }
//    }
//}

void updateVoxelMapOMP(const std::vector<pointWithCov> &input_points,
                    const float voxel_size, const int max_layer,
                    const std::vector<int> &layer_point_size,
                    const int max_points_size, const int max_cov_points_size,
                    const float planer_threshold,
                    std::unordered_map<VOXEL_LOC, OctoTree *> &feat_map) {

  std::unordered_map<VOXEL_LOC, vector<pointWithCov>> position_index_map;
  int insert_count = 0, update_count = 0;
  uint plsize = input_points.size();


  double t_insert_start = omp_get_wtime();
  for (uint i = 0; i < plsize; i++) {
    const pointWithCov p_v = input_points[i];
    // 计算voxel坐标
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_v.point[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    // 如果点的位置已经存在voxel 那么就更新点的位置 否则创建新的voxel
    if (iter != feat_map.end()) {
      // 更新的点总是很多 先缓存 再延迟并行更新
      update_count++;
      position_index_map[position].push_back(p_v);
    } else {
      // 插入的点总是少的 直接单线程插入
      // 保存position位置对应的点
      insert_count++;
      OctoTree *octo_tree =
          new OctoTree(max_layer, 0, layer_point_size, max_points_size,
                       max_cov_points_size, planer_threshold);
      feat_map[position] = octo_tree;
      feat_map[position]->quater_length_ = voxel_size / 4;
      feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
      feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
      feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
      feat_map[position]->UpdateOctoTree(p_v);
    }
  }
  double t_insert_end = omp_get_wtime();
  double t_update_start = omp_get_wtime();
    // 并行延迟更新
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for default(none) shared(position_index_map, feat_map)
#endif
    for (size_t b = 0; b < position_index_map.bucket_count(); b++) {
        // 先遍历bucket 理想情况下bucket一般只有一个元素 这样还是相当于完全并行的遍历position_index_map
        // XXX 需要确定最坏情况下bucket的元素数量
        for (auto bi = position_index_map.begin(b); bi != position_index_map.end(b); bi++) {
            VOXEL_LOC position = bi->first;
            for (const pointWithCov &p_v:bi->second) {
                feat_map[position]->UpdateOctoTree(p_v);
            }
        }
    }
    double t_update_end = omp_get_wtime();
//    std::printf("Insert & store:  %.4fs  Update:  %.4fs  |  Inserted: %d  Updated: %d \n",
//                t_insert_end - t_insert_start,
//                t_update_end - t_update_start,
//                insert_count, update_count);
}


void build_single_residual(const pointWithCov &pv, const OctoTree *current_octo,
                           const int current_layer, const int max_layer,
                           const double sigma_num, bool &is_sucess,
                           double &prob, ptpl &single_ptpl) {
  double radius_k = 3;
  Eigen::Vector3d p_w = pv.point_world;
  // 如果当前voxel是平面 则构建voxel block 否则递归搜索当前voxel的leaves 直到找到平面
  // XXX 如果不是平面是不是可以在构建的时候直接剪掉？
  if (current_octo->plane_ptr_->is_plane) {
      Plane &plane = *current_octo->plane_ptr_;
//    // 判断平面和当前视角是不是锐角，判断是否为当前视角的观测
//    // 计算normal与点-点夹角
//    double cos_val = plane.calc_normal_viewpoint_cos(current_state_point.pos);
//    // 判断是锐角还是钝角 如果是钝角, 说明是错误匹配, 需要剔除
//    if(cos_val < 0) {
//        // return;
//        // XXX 临时实现: 如果是反方向的平面特征，用平面2来尝试匹配
//        if (current_octo->plane_ptr_2_->is_plane) {
//            plane = *current_octo->plane_ptr_2_;
//        } else {
//            return;
//        }
//    }

    // HACK 这个是LiDAR点到地图plane的点面距离
    float dis_to_plane =
        fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
             plane.normal(2) * p_w(2) + plane.d);
    // HACK 这个是LiDAR点到构建地图plane的点簇中心的距离
    float dis_to_center =
        (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
        (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
        (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
    // HACK 差值是 点在平面上投影 与 平面点簇中心的距离
    // HACK 目的是不要用距离平面点簇太远的点来做残差，因为估计的平面在这些远点的位置可能不满足平面假设了
    // HACK 因为将点划分进voxel的时候只用了第一层voxel 这个voxel可能比较大 遍历到的这个子voxel距离点可能还比较远
    float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

    if (range_dis <= radius_k * plane.radius) {
      // 计算点面距离的方差
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = p_w - plane.center;
      J_nq.block<1, 3>(0, 3) = -plane.normal;
      double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
      sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
      // 只选择距离在3sigma之内的匹配
      if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
        is_sucess = true;
        // 求对应正态分布的概率密度值 意思是落在当前平面有多大可能性 注意这个分布的u=0 所以直接用dis_to_plane平方来求
        // HACK 这里比fast lio和任何loam系的都要clever得多
        double this_prob = 1.0 / (sqrt(sigma_l)) *
                           exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
        // 在递归的过程中不断比较 最后保留一个最大概率对应的residual
        if (this_prob > prob) {
          prob = this_prob;
          single_ptpl.point = pv.point;
          single_ptpl.point_world = pv.point_world;
          single_ptpl.plane_cov = plane.plane_cov;
          single_ptpl.normal = plane.normal;
          single_ptpl.center = plane.center;
          single_ptpl.d = plane.d;
          single_ptpl.layer = current_layer;
          single_ptpl.cov_lidar = pv.cov_lidar;
        }
        return;
      } else {
        // is_sucess = false;
        return;
      }
    } else {
      // is_sucess = false;
      return;
    }
  } else {
    if (current_layer < max_layer) {
      // 遍历当前节点的所有叶子 往下递归
      for (size_t leafnum = 0; leafnum < 8; leafnum++) {
        if (current_octo->leaves_[leafnum] != nullptr) {

          OctoTree *leaf_octo = current_octo->leaves_[leafnum];
          build_single_residual(pv, leaf_octo, current_layer + 1, max_layer,
                                sigma_num, is_sucess, prob, single_ptpl);
        }
      }
      return;
    } else {
      // is_sucess = false;
      return;
    }
  }
}

void GetUpdatePlane(const OctoTree *current_octo, const int pub_max_voxel_layer,
                    std::vector<Plane> &plane_list) {
  if (current_octo->layer_ > pub_max_voxel_layer) {
    return;
  }
  if (current_octo->plane_ptr_->is_update) {
    plane_list.push_back(*current_octo->plane_ptr_);
  }
//  // 平面2
//  if (current_octo->plane_ptr_2_->is_update) {
//    plane_list.push_back(*current_octo->plane_ptr_2_);
//  }
  if (current_octo->layer_ < current_octo->max_layer_) {
    if (!current_octo->plane_ptr_->is_plane) {
      for (size_t i = 0; i < 8; i++) {
        if (current_octo->leaves_[i] != nullptr) {
          GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer,
                         plane_list);
        }
      }
    }
  }
  return;
}

// void BuildResidualListTBB(const unordered_map<VOXEL_LOC, OctoTree *>
// &voxel_map,
//                           const double voxel_size, const double sigma_num,
//                           const int max_layer,
//                           const std::vector<pointWithCov> &pv_list,
//                           std::vector<ptpl> &ptpl_list,
//                           std::vector<Eigen::Vector3d> &non_match) {
//   std::mutex mylock;
//   ptpl_list.clear();
//   std::vector<ptpl> all_ptpl_list(pv_list.size());
//   std::vector<bool> useful_ptpl(pv_list.size());
//   std::vector<size_t> index(pv_list.size());
//   for (size_t i = 0; i < index.size(); ++i) {
//     index[i] = i;
//     useful_ptpl[i] = false;
//   }
//   std::for_each(
//       std::execution::par_unseq, index.begin(), index.end(),
//       [&](const size_t &i) {
//         pointWithCov pv = pv_list[i];
//         float loc_xyz[3];
//         for (int j = 0; j < 3; j++) {
//           loc_xyz[j] = pv.point_world[j] / voxel_size;
//           if (loc_xyz[j] < 0) {
//             loc_xyz[j] -= 1.0;
//           }
//         }
//         VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                            (int64_t)loc_xyz[2]);
//         auto iter = voxel_map.find(position);
//         if (iter != voxel_map.end()) {
//           OctoTree *current_octo = iter->second;
//           ptpl single_ptpl;
//           bool is_sucess = false;
//           double prob = 0;
//           build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
//                                 is_sucess, prob, single_ptpl);
//           if (!is_sucess) {
//             VOXEL_LOC near_position = position;
//             if (loc_xyz[0] > (current_octo->voxel_center_[0] +
//                               current_octo->quater_length_)) {
//               near_position.x = near_position.x + 1;
//             } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
//                                      current_octo->quater_length_)) {
//               near_position.x = near_position.x - 1;
//             }
//             if (loc_xyz[1] > (current_octo->voxel_center_[1] +
//                               current_octo->quater_length_)) {
//               near_position.y = near_position.y + 1;
//             } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
//                                      current_octo->quater_length_)) {
//               near_position.y = near_position.y - 1;
//             }
//             if (loc_xyz[2] > (current_octo->voxel_center_[2] +
//                               current_octo->quater_length_)) {
//               near_position.z = near_position.z + 1;
//             } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
//                                      current_octo->quater_length_)) {
//               near_position.z = near_position.z - 1;
//             }
//             auto iter_near = voxel_map.find(near_position);
//             if (iter_near != voxel_map.end()) {
//               build_single_residual(pv, iter_near->second, 0, max_layer,
//                                     sigma_num, is_sucess, prob, single_ptpl);
//             }
//           }
//           if (is_sucess) {

//             mylock.lock();
//             useful_ptpl[i] = true;
//             all_ptpl_list[i] = single_ptpl;
//             mylock.unlock();
//           } else {
//             mylock.lock();
//             useful_ptpl[i] = false;
//             mylock.unlock();
//           }
//         }
//       });
//   for (size_t i = 0; i < useful_ptpl.size(); i++) {
//     if (useful_ptpl[i]) {
//       ptpl_list.push_back(all_ptpl_list[i]);
//     }
//   }
// }

void BuildResidualListOMP(const unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                          const double voxel_size, const double sigma_num,
                          const int max_layer,
                          const std::vector<pointWithCov> &pv_list,
                          std::vector<ptpl, Eigen::aligned_allocator<ptpl>> &ptpl_list,
                          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &non_match) {
  std::mutex mylock;
  ptpl_list.clear();
  std::vector<ptpl, Eigen::aligned_allocator<ptpl>> all_ptpl_list(pv_list.size());
  std::vector<bool> useful_ptpl(pv_list.size());
  std::vector<size_t> index(pv_list.size());
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
    useful_ptpl[i] = false;
  }
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  // 这个文章在实现的时候 第一层voxel并没有严格作为根节点，而是现有一个层次的结构，这样方便管理
  for (int i = 0; i < index.size(); i++) {
    pointWithCov pv = pv_list[i];
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pv.point_world[j] / voxel_size;
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);
    // 查找当前点所属的voxel
    auto iter = voxel_map.find(position);

    if (iter != voxel_map.end()) {
      OctoTree *current_octo = iter->second;
      ptpl single_ptpl;
      bool is_sucess = false;
      double prob = 0;
      // 找到之后构建residual 返回值是single_ptpl 包含了与点匹配的平面的所有信息
      build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
                            is_sucess, prob, single_ptpl);
      // 如果不成功 根据当前点偏离voxel的程度 查找临近的voxel
      // HACK 这里是为了处理点落在两个voxel边界的情况 可能真实匹配的平面在临近的voxel中
      if (!is_sucess) {
        VOXEL_LOC near_position = position;
        if (loc_xyz[0] >
            (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
          near_position.x = near_position.x + 1;
        } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                                 current_octo->quater_length_)) {
          near_position.x = near_position.x - 1;
        }
        if (loc_xyz[1] >
            (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
          near_position.y = near_position.y + 1;
        } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                                 current_octo->quater_length_)) {
          near_position.y = near_position.y - 1;
        }
        if (loc_xyz[2] >
            (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
          near_position.z = near_position.z + 1;
        } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                                 current_octo->quater_length_)) {
          near_position.z = near_position.z - 1;
        }
        auto iter_near = voxel_map.find(near_position);
        if (iter_near != voxel_map.end()) {
          build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
                                is_sucess, prob, single_ptpl);
        }
      }

      // 所有点的匹配结果储存到list中
      if (is_sucess) {

        mylock.lock();
        useful_ptpl[i] = true;
        all_ptpl_list[i] = single_ptpl;
        mylock.unlock();
      } else {
        mylock.lock();
        useful_ptpl[i] = false;
        mylock.unlock();
      }
    }
  }
  for (size_t i = 0; i < useful_ptpl.size(); i++) {
    if (useful_ptpl[i]) {
      ptpl_list.push_back(all_ptpl_list[i]);
    }
  }
}

//void BuildResidualListNormal(
//    const unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
//    const double voxel_size, const double sigma_num, const int max_layer,
//    const std::vector<pointWithCov> &pv_list, std::vector<ptpl, Eigen::aligned_allocator<ptpl>> &ptpl_list,
//    std::vector<Eigen::Vector3d> &non_match) {
//  ptpl_list.clear();
//  std::vector<size_t> index(pv_list.size());
//  for (size_t i = 0; i < pv_list.size(); ++i) {
//    pointWithCov pv = pv_list[i];
//    float loc_xyz[3];
//    for (int j = 0; j < 3; j++) {
//      loc_xyz[j] = pv.point_world[j] / voxel_size;
//      if (loc_xyz[j] < 0) {
//        loc_xyz[j] -= 1.0;
//      }
//    }
//    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
//                       (int64_t)loc_xyz[2]);
//    auto iter = voxel_map.find(position);
//    if (iter != voxel_map.end()) {
//      OctoTree *current_octo = iter->second;
//      ptpl single_ptpl;
//      bool is_sucess = false;
//      double prob = 0;
//      build_single_residual(pv, current_octo, 0, max_layer, sigma_num,
//                            is_sucess, prob, single_ptpl);
//
//      if (!is_sucess) {
//        VOXEL_LOC near_position = position;
//        if (loc_xyz[0] >
//            (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
//          near_position.x = near_position.x + 1;
//        } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
//                                 current_octo->quater_length_)) {
//          near_position.x = near_position.x - 1;
//        }
//        if (loc_xyz[1] >
//            (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
//          near_position.y = near_position.y + 1;
//        } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
//                                 current_octo->quater_length_)) {
//          near_position.y = near_position.y - 1;
//        }
//        if (loc_xyz[2] >
//            (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
//          near_position.z = near_position.z + 1;
//        } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
//                                 current_octo->quater_length_)) {
//          near_position.z = near_position.z - 1;
//        }
//        auto iter_near = voxel_map.find(near_position);
//        if (iter_near != voxel_map.end()) {
//          build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num,
//                                is_sucess, prob, single_ptpl);
//        }
//      }
//      if (is_sucess) {
//        ptpl_list.push_back(single_ptpl);
//      } else {
//        non_match.push_back(pv.point_world);
//      }
//    }
//  }
//}

void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec,
                     const Eigen::Vector3d &z_vec,
                     geometry_msgs::Quaternion &q) {

  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0),
      z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  eq.normalize();
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q) {
  Eigen::Vector3d x_body = vec;
  Eigen::Vector3d y_body(1, 1, 0);
  if (x_body(2) != 0) {
    y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
  } else {
    if (x_body(1) != 0) {
      y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
    } else {
      y_body(0) = 0;
    }
  }
  y_body.normalize();
  Eigen::Vector3d z_body = x_body.cross(y_body);
  Eigen::Matrix3d rot;

  rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
      z_body(0), z_body(1), z_body(2);
  Eigen::Matrix3d rotation = rot.transpose();
  if (axis == 2) {
    Eigen::Matrix3d rot_inc;
    rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    rotation = rotation * rot_inc;
  }
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

//void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub,
//                    const std::string plane_ns, const Plane &single_plane,
//                    const float alpha, const Eigen::Vector3d rgb) {
//  visualization_msgs::Marker plane;
//  plane.header.frame_id = "camera_init";
//  plane.header.stamp = ros::Time();
//  plane.ns = plane_ns;
//  plane.id = single_plane.id;
//  plane.type = visualization_msgs::Marker::CYLINDER;
//  plane.action = visualization_msgs::Marker::ADD;
//  plane.pose.position.x = single_plane.center[0];
//  plane.pose.position.y = single_plane.center[1];
//  plane.pose.position.z = single_plane.center[2];
//  geometry_msgs::Quaternion q;
//  CalcVectQuation(single_plane.x_normal, single_plane.y_normal,
//                  single_plane.normal, q);
//  plane.pose.orientation = q;
//  plane.scale.x = 3 * sqrt(single_plane.max_eigen_value);
//  plane.scale.y = 3 * sqrt(single_plane.mid_eigen_value);
//  plane.scale.z = 2 * sqrt(single_plane.min_eigen_value);
//  plane.color.a = alpha;
//  plane.color.r = rgb(0);
//  plane.color.g = rgb(1);
//  plane.color.b = rgb(2);
//  plane.lifetime = ros::Duration();
//  plane_pub.markers.push_back(plane);
//}

void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub,
                    const std::string plane_ns, const Plane &single_plane,
                    const float alpha, const Eigen::Vector3d rgb) {
    visualization_msgs::Marker plane;
    plane.header.frame_id = "camera_init";
    plane.header.stamp = ros::Time();
    plane.ns = plane_ns;
    plane.id = single_plane.id;
    plane.type = visualization_msgs::Marker::ARROW;
    plane.action = visualization_msgs::Marker::ADD;
    plane.pose.position.x = single_plane.center[0];
    plane.pose.position.y = single_plane.center[1];
    plane.pose.position.z = single_plane.center[2];

//    // 计算normal与点-点夹角
//    // 注意 这里应该是平面中心于当前帧位置的距离;
//    //  而且注意向量的方向，想求的是(法线中心-法线末端，法线中心-原点)这两个向量的夹角
//    Eigen::Vector3d point_center = current_state_point.pos - single_plane.center;
//    Eigen::Vector3d normal = single_plane.normal;
//    double cos_val = point_center.dot(normal) / (point_center.norm() * normal.norm()); //角度cos值
//    // 判断是锐角还是钝角 如果是钝角, 则将法线反向， 保证法线是朝向LiDAR的
//    if(cos_val < 0) {
//        normal = -normal;
//    }

    // 姿态角是法线相对于+X的夹角
    Eigen::Vector3d nx1 = single_plane.normal.normalized();
//     Eigen::Vector3d nx1 = normal.normalized();
    Eigen::Vector3d nx2{1.0, 0, 0};
    // 计算T__nx1__o__nx2
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(nx2, nx1);

//    geometry_msgs::Quaternion q;
//    CalcVectQuation(single_plane.x_normal, single_plane.y_normal,
//                    single_plane.normal, q);
    plane.pose.orientation.w = q.w();
    plane.pose.orientation.x = q.x();
    plane.pose.orientation.y = q.y();
    plane.pose.orientation.z = q.z();
    plane.scale.x = 0.5;
    plane.scale.y = 0.05;
    plane.scale.z = 0.05;
    plane.color.a = alpha;

//    plane.color.r = rgb(0);
//    plane.color.g = rgb(1);
//    plane.color.b = rgb(2);
    if(single_plane.update_enable){
        plane.color.r = 0.0;
        plane.color.g = 1.0;
        plane.color.b = 0.0;
    } else {
        plane.color.r = 1.0;
        plane.color.g = 0.0;
        plane.color.b = 0.0;
    }
    plane.lifetime = ros::Duration();
    plane_pub.markers.push_back(plane);
}


void pubVoxelMap(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 const int pub_max_voxel_layer,
                 const ros::Publisher &plane_map_pub) {
  double max_trace = 0.25;
  double pow_num = 0.2;
  ros::Rate loop(500);
  float use_alpha = 0.8;
  visualization_msgs::MarkerArray voxel_plane;
  voxel_plane.markers.reserve(1000000);
  std::vector<Plane> pub_plane_list;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    GetUpdatePlane(iter->second, pub_max_voxel_layer, pub_plane_list);
  }
  for (size_t i = 0; i < pub_plane_list.size(); i++) {
    V3D plane_cov = pub_plane_list[i].plane_cov.block<3, 3>(0, 0).diagonal();
    double trace = plane_cov.sum();
    if (trace >= max_trace) {
      trace = max_trace;
    }
    trace = trace * (1.0 / max_trace);
    trace = pow(trace, pow_num);
    uint8_t r, g, b;
    mapJet(trace, 0, 1, r, g, b);
    Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
    double alpha;
    if (pub_plane_list[i].is_plane) {
      alpha = use_alpha;
    } else {
      alpha = 0;
    }
    if (alpha!= 0){
        pubSinglePlane(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
    }
  }
  plane_map_pub.publish(voxel_plane);
  loop.sleep();
}


void GetPointsInVoxel(const OctoTree *current_octo, const int pub_max_voxel_layer,
                    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> &points_list) {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
//    if (current_octo->plane_ptr_->is_update) {
    if (current_octo->plane_ptr_->is_plane) {
        for(auto p: current_octo->temp_points_){
            pcl::PointXYZRGB p_color;
            p_color.x = p.point.x();
            p_color.y = p.point.y();
            p_color.z = p.point.z();
//            std::printf("%f %f %f, %f %f %f, %f %f %f\n",
//                        p_color.x, p_color.y, p_color.z,
//                        p.point.x(),  p.point.y(),  p.point.z(),
//                        float(p.point.x()),  float(p.point.y()),  float(p.point.z())
//                        );
            p_color.r = current_octo->colors[0];
            p_color.g = current_octo->colors[1];
            p_color.b = current_octo->colors[2];
            points_list.push_back(p_color);
        }
    }
//    // 发布第二个平面
//    if (current_octo->plane_ptr_2_->is_plane) {
//        // 每个voxel对应一种随机颜色 区别于第一平面
//        std::vector<unsigned int> colors;
//        colors.push_back(static_cast<unsigned int>(rand() % 256));
//        colors.push_back(static_cast<unsigned int>(rand() % 256));
//        colors.push_back(static_cast<unsigned int>(rand() % 256));
//        for(auto p: current_octo->temp_points_2_){
//            pcl::PointXYZRGB p_color;
//            p_color.x = p.point.x();
//            p_color.y = p.point.y();
//            p_color.z = p.point.z();
////            std::printf("%f %f %f, %f %f %f, %f %f %f\n",
////                        p_color.x, p_color.y, p_color.z,
////                        p.point.x(),  p.point.y(),  p.point.z(),
////                        float(p.point.x()),  float(p.point.y()),  float(p.point.z())
////                        );
//            p_color.r = colors[0];
//            p_color.g = colors[1];
//            p_color.b = colors[2];
//            points_list.push_back(p_color);
//        }
//    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetPointsInVoxel(current_octo->leaves_[i], pub_max_voxel_layer,
                                   points_list);
                }
            }
        }
    }
    return;
}

void pubColoredVoxels(const std::unordered_map<VOXEL_LOC, OctoTree *> &voxel_map,
                 const int pub_max_voxel_layer,
                 const ros::Publisher &voxel_map_pub, double lidar_end_time) {

    ros::Rate loop(500);
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> pub_points_list;
    // 获取每个voxel中的点
    for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
        GetPointsInVoxel(iter->second, pub_max_voxel_layer, pub_points_list);
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // colored_point_cloud.points.swap(pub_points_list);
    for (auto p:pub_points_list){
        colored_point_cloud->push_back(p);
    }

//    // 下采样可视化点云
//    pcl::PointCloud<pcl::PointXYZRGB> colored_point_cloud_down;
//    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterVis;
//    downSizeFilterVis.setInputCloud(colored_point_cloud);
//    downSizeFilterVis.setLeafSize(0.3, 0.3, 0.3);
//    downSizeFilterVis.filter(colored_point_cloud_down);

    sensor_msgs::PointCloud2 voxel_cloud2;
//    pcl::toROSMsg(colored_point_cloud_down, voxel_cloud2);
    pcl::toROSMsg(*colored_point_cloud, voxel_cloud2);
    voxel_cloud2.header.frame_id = "camera_init";
    voxel_cloud2.header.stamp = ros::Time().fromSec(lidar_end_time);
    voxel_map_pub.publish(voxel_cloud2);
    loop.sleep();
}

M3D calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc)
{
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
      pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  return direction * range_var * direction.transpose() +
        A * direction_var * A.transpose();
};

#endif