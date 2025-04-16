#include <Eigen/Dense>

Eigen::Vector3d spring(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d sphere(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d cube(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d viscosity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d friction(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d line(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d curve(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
Eigen::Vector3d extra(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
